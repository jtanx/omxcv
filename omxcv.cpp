/**
 * @file omxcv.cpp
 * @brief Routines to perform hardware accelerated H.264 encoding on the RPi.
 */

#include "omxcv.h"
#include "omxcv-impl.h"
using namespace omxcv;

using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>

#define TIMEDIFF(start) (duration_cast<milliseconds>(steady_clock::now() - start).count())
#define CHECKED(c, v) if ((c)) throw std::invalid_argument(v)

/**
 * Initialise LibAVCodec output muxer.
 * @return true iff initialised.
 */
bool OmxCvImpl::lav_init() {
    av_register_all();
    av_log_set_level(AV_LOG_INFO);

    AVOutputFormat *fmt = av_guess_format(NULL, m_filename.c_str(), NULL);
    if (fmt == NULL) {
        return false;
    }

    m_mux_ctx = avformat_alloc_context();
    if (m_mux_ctx == NULL) {
        return false;
    }
    m_mux_ctx->debug = 1;
    m_mux_ctx->start_time_realtime = time(NULL); //NOW
    m_mux_ctx->start_time = AV_NOPTS_VALUE;
    m_mux_ctx->oformat = fmt;
    snprintf(m_mux_ctx->filename, sizeof(m_mux_ctx->filename), "%s", m_filename.c_str());

    m_video_stream = avformat_new_stream(m_mux_ctx, NULL);
    if (m_video_stream == NULL) {
        avformat_free_context(m_mux_ctx);
        return false;
    }

    m_video_stream->codec->width = m_width;
    m_video_stream->codec->height = m_height;
    m_video_stream->codec->codec_id = AV_CODEC_ID_H264;
    m_video_stream->codec->codec_type = AVMEDIA_TYPE_VIDEO;
    m_video_stream->codec->bit_rate = m_bitrate;
    m_video_stream->codec->time_base.num = 1;
    m_video_stream->codec->time_base.den = 1000;
    m_video_stream->codec->pix_fmt = AV_PIX_FMT_YUV420P;

    m_video_stream->time_base.num = 1;
    m_video_stream->time_base.den = 1000;

    m_video_stream->start_time = AV_NOPTS_VALUE;

    m_video_stream->codec->sample_aspect_ratio.num = m_video_stream->sample_aspect_ratio.num;
    m_video_stream->codec->sample_aspect_ratio.den = m_video_stream->sample_aspect_ratio.den;

    if (avio_open(&m_mux_ctx->pb, m_filename.c_str(), AVIO_FLAG_WRITE) < 0) {
        avcodec_close(m_video_stream->codec);
        avformat_free_context(m_mux_ctx);
        return 0;
    }

    if (m_mux_ctx->oformat->flags & AVFMT_GLOBALHEADER) {
        m_video_stream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;
    }

    return true;
}

/**
 * Dumps the CodecPrivate data for the Matroska/MP4 container.
 * Note this is the same format as required for MP4 containers - in other words,
 * the AVCC format (AVCDecoderConfigurationRecord structure). Some values have
 * been hard-coded for simplicity's sake.
 * @return true iff CodecPrivate (extradata) was updated.
 */
bool OmxCvImpl::dump_codec_private() {
    //Example CodecPrivate: 01 64 00 1e ff e1 00 0e 27 64 00 1e ac 2b 40 50 1e d0 0f 12 26 a0 01 00 05 28 ee 02 5c b0
    //See also: http://lists.matroska.org/pipermail/matroska-devel/2012-April/004196.html
    //And: http://stackoverflow.com/questions/24884827/possible-locations-for-sequence-picture-parameter-sets-for-h-264-stream
    //Note: lengthSizeMinusOne is normally 3 (4 bytes to specify NALU length)
    //But: Annex B NALU signature is either 3 or 4 bytes long.
    //However: Luckily for us, the RPi OMX encoder appears to always return
    //NALUs with 4 byte headers. So we're OK! If we get one with a 3 byte header
    //then we have to increase the buffer size by 1 to accomodate the size.
    assert(m_sps_length > 2);
    uint8_t adcr[] = {
        1, //configurationVersion
        m_sps[1], //AVCProfileIndication
        m_sps[2], //profile_compatibility
        m_sps[3], //AVCLevelIndication
        0xfc | 3, //lengthSizeMinusOne;
        0xe0 | 1 //numOfSequenceParameterSets; 0xe0 for reserved bits. We have one SPS.
    };

    AVCodecContext *c = m_video_stream->codec;
    av_free(c->extradata);

    c->profile = m_sps[1]; //Ususally 'High' profile
    c->level = m_sps[3]; //Usually Level 3.0

    c->extradata_size = sizeof(adcr) + 2 + m_sps_length + 1 + 2 + m_pps_length;
    c->extradata = static_cast<uint8_t*>(av_malloc(c->extradata_size));
    if (c->extradata) {
        memcpy(c->extradata, &adcr, sizeof(adcr));
        c->extradata[sizeof(adcr)] = m_sps_length >> 8;
        c->extradata[sizeof(adcr)+1] = m_sps_length & 0xFF;
        memcpy(c->extradata+sizeof(adcr)+2, m_sps, m_sps_length);
        c->extradata[sizeof(adcr)+2+m_sps_length] = 1; //PPS count
        c->extradata[sizeof(adcr)+2+m_sps_length+1] = m_pps_length >> 8;
        c->extradata[sizeof(adcr)+2+m_sps_length+2] = m_pps_length & 0xFF;
        memcpy(c->extradata+sizeof(adcr)+2+m_sps_length+3, m_pps, m_pps_length);
    } else {
        perror("Extradata allocation failed");
        return false;
    }

    //Write the file header.
    avformat_write_header(m_mux_ctx, NULL);
    //Print info about this format
    av_dump_format(m_mux_ctx, 0, m_filename.c_str(), 1);
    return true;
}

/**
 * Constructor.
 * @param [in] name The file to save to.
 * @param [in] width The video width.
 * @param [in] height The video height.
 * @param [in] bitrate The bitrate, in Kbps.
 * @param [in] fpsnum The FPS numerator.
 * @param [in] fpsden The FPS denominator.
 */
OmxCvImpl::OmxCvImpl(const char *name, int width, int height, int bitrate, int fpsnum, int fpsden)
: m_width(width)
, m_height(height)
, m_stride(((width + 31) & ~31) * 3)
, m_bitrate(bitrate)
, m_sps(nullptr)
, m_pps(nullptr)
, m_sps_length(0)
, m_pps_length(0)
, m_initted_header(false)
, m_filename(name)
, m_stop{false}
{
    int ret;
    bcm_host_init();
    OMX_Init();

    if (fpsden <= 0 || fpsnum <= 0) {
        fpsden = 1;
        fpsnum = 25;
    }
    m_fpsnum = fpsnum;
    m_fpsden = fpsden;
    //Initialise the scaler and output file
    CHECKED(!lav_init(), "Failed to initialise LibAVFormat.");

    m_ilclient = ilclient_init();
    CHECKED(m_ilclient == NULL, "ILClient initialisation failed.");

    ret = ilclient_create_component(m_ilclient, &m_encoder_component,
            (char*)"video_encode",
            (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS |
            ILCLIENT_ENABLE_INPUT_BUFFERS | ILCLIENT_ENABLE_OUTPUT_BUFFERS));
    CHECKED(ret != 0, "ILCient video_encode component creation failed.");

    //Set input definition to the encoder
    OMX_PARAM_PORTDEFINITIONTYPE def = {0};
    def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    def.nVersion.nVersion = OMX_VERSION;
    def.nPortIndex = OMX_ENCODE_PORT_IN;
    ret = OMX_GetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamPortDefinition, &def);
    CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed for encode port in.");

    def.format.video.nFrameWidth = m_width;
    def.format.video.nFrameHeight = m_height;
    def.format.video.xFramerate = 25 << 16;
    //Must be a multiple of 16
    def.format.video.nSliceHeight = (m_height + 15) & ~15;
    //Must be a multiple of 32
    def.format.video.nStride = m_stride;
    def.format.video.eColorFormat =  OMX_COLOR_Format24bitBGR888; //OMX_COLOR_Format32bitABGR8888;//OMX_COLOR_FormatYUV420PackedPlanar;
    //Must be manually defined to ensure sufficient size if stride needs to be rounded up to multiple of 32.
    def.nBufferSize = def.format.video.nStride * def.format.video.nSliceHeight;

    ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamPortDefinition, &def);
    CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed for input format definition.");

    //Set the output format of the encoder
    OMX_VIDEO_PARAM_PORTFORMATTYPE format = {0};
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = OMX_ENCODE_PORT_OUT;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;

    ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamVideoPortFormat, &format);
    CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed for setting encoder output format.");

    //Set the encoding bitrate
    OMX_VIDEO_PARAM_BITRATETYPE bitrate_type = {0};
    bitrate_type.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    bitrate_type.nVersion.nVersion = OMX_VERSION;
    bitrate_type.eControlRate = OMX_Video_ControlRateVariable;
    bitrate_type.nTargetBitrate = bitrate * 1000;
    bitrate_type.nPortIndex = OMX_ENCODE_PORT_OUT;
    ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamVideoBitrate, &bitrate_type);
    CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed for setting encoder bitrate.");

    ret = ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    CHECKED(ret != 0, "ILClient failed to change encoder to idle state.");
    ret = ilclient_enable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_IN, NULL, NULL, NULL);
    CHECKED(ret != 0, "ILClient failed to enable input buffers.");
    ret = ilclient_enable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_OUT, NULL, NULL, NULL);
    CHECKED(ret != 0, "ILClient failed to enable output buffers.");
    ret = ilclient_change_component_state(m_encoder_component, OMX_StateExecuting);
    CHECKED(ret != 0, "ILClient failed to change encoder to executing stage.");

    //Start the worker thread for dumping the encoded data
    m_input_worker = std::thread(&OmxCvImpl::input_worker, this);
}

/**
 * Destructor.
 * @return Return_Description
 */
OmxCvImpl::~OmxCvImpl() {
    m_stop = true;
    m_input_signaller.notify_one();
    m_input_worker.join();

    //Free the SPS and PPS headers.
    delete [] m_sps;
    delete [] m_pps;

    //Teardown similar to hello_encode
    ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    ilclient_disable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_IN, NULL, NULL, NULL);
    ilclient_disable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_OUT, NULL, NULL, NULL);

    //ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    ilclient_change_component_state(m_encoder_component, OMX_StateLoaded);

    COMPONENT_T *list[] = {m_encoder_component, NULL};
    ilclient_cleanup_components(list);
    ilclient_destroy(m_ilclient);

    //Close the output file
    av_write_trailer(m_mux_ctx);
    avcodec_close(m_video_stream->codec);
    avio_close(m_mux_ctx->pb);
    avformat_free_context(m_mux_ctx);
}

/**
 * Input encoding routine.
 */
void OmxCvImpl::input_worker() {
    std::unique_lock<std::mutex> lock(m_input_mutex);
    OMX_BUFFERHEADERTYPE *in = ilclient_get_input_buffer(m_encoder_component, OMX_ENCODE_PORT_IN, 1);
    OMX_BUFFERHEADERTYPE *out = ilclient_get_output_buffer(m_encoder_component, OMX_ENCODE_PORT_OUT, 1);

    //FILE *fp = fopen("test.yuv", "wb");
    while (true) {
        m_input_signaller.wait(lock, [this]{return m_stop || m_input_queue.size() > 0;});
        if (m_stop) {
            if (m_input_queue.size() > 0) {
                printf("Stop acknowledged but need to flush the input buffer (%d)...\n", m_input_queue.size());
            } else {
                break;
            }
        }

        //auto proc_start = steady_clock::now();
        std::pair<cv::Mat, int64_t> frame = m_input_queue.front();
        cv::Mat mat = frame.first;
        m_input_queue.pop_front();
        lock.unlock();

        if (in == NULL) {
            printf("NO INPUT BUFFER");
        } else {
            //auto conv_start = steady_clock::now();
            cv::Mat omat(mat.rows, mat.cols, CV_8UC3, in->pBuffer, m_stride);
            cv::cvtColor(mat, omat, CV_BGR2RGB);
            in->nFilledLen = in->nAllocLen;

            //static int framecounter = 0;
            //printf("BGR2RGB time (ms): %-3d [%d]\r", (int)TIMEDIFF(conv_start), ++framecounter);
            //fflush(stdout);

            OMX_EmptyThisBuffer(ILC_GET_HANDLE(m_encoder_component), in);
            //printf("Encoding time (ms): %d [%d]\r", (int)TIMEDIFF(conv_start), ++framecounter);
            do {
                OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component), out);
                out = ilclient_get_output_buffer(m_encoder_component, OMX_ENCODE_PORT_OUT, 1);
            } while (!write_data(out, frame.second));
            in = ilclient_get_input_buffer(m_encoder_component, OMX_ENCODE_PORT_IN, 1);
        }

        lock.lock();
        //printf("Total processing time (ms): %d\n", (int)TIMEDIFF(proc_start));
    }

    //fclose(fp);
    in->nFilledLen = 0;
    OMX_EmptyThisBuffer(ILC_GET_HANDLE(m_encoder_component), in);
    OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component), out);
}

/**
 * Output muxing routine.
 * @param [in] out Buffer to be saved.
 * @param [in] timestamp Timestamp of this buffer.
 * @return true if buffer was saved.
 */
bool OmxCvImpl::write_data(OMX_BUFFERHEADERTYPE *out, int64_t timestamp) {
    bool ret = false;

    if (out != NULL) {
        if (out->nFilledLen > 4) {
            AVPacket pkt;
            ret = true;

            //Check for an SPS/PPS header
            //Signature is either 00 00 00 01 or 00 00 01; we need the former.
            if (out->nFilledLen > 5 && out->pBuffer[2] == 0) {
                uint8_t naltype = out->pBuffer[4] & 0x1f;
                if (m_initted_header) {
                    if (naltype == 7 || naltype == 8) {
                        ret = false;
                    }
                } else {
                    if (naltype == 7) { //SPS
                        delete [] m_sps;
                        m_sps_length = out->nFilledLen - 4;
                        m_sps = new uint8_t[m_sps_length];
                        //Strip the NAL header.
                        memcpy(m_sps, out->pBuffer+4, m_sps_length);
                        ret = false;
                    } else if (naltype == 8) {
                        delete [] m_pps;
                        m_pps_length = out->nFilledLen - 4;
                        m_pps = new uint8_t[m_pps_length];
                        //Strip the NAL header.
                        memcpy(m_pps, out->pBuffer+4, m_pps_length);
                        ret = false;
                    }

                    if (m_sps && m_pps) {
                        if (dump_codec_private()) {
                            m_initted_header = true;
                        }
                    }
                }
            }

            if (ret) {
                av_init_packet(&pkt);
                pkt.stream_index = m_video_stream->index;
                pkt.pts = timestamp;

                //AVCC format.
                if (out->pBuffer[2] == 1) {
                    //Slow fallback. But this never appears to happen.
                    out->nFilledLen++;
                    pkt.data = static_cast<uint8_t*>(malloc(out->nFilledLen));
                    memcpy(pkt.data+1, out->pBuffer, out->nFilledLen-1);
                } else {
                    pkt.data = out->pBuffer;
                }

                pkt.size = out->nFilledLen;
                out->nFilledLen -= 4;
                pkt.data[0] = (out->nFilledLen >> 24) & 0xFF;
                pkt.data[1] = (out->nFilledLen >> 16) & 0xFF;
                pkt.data[2] = (out->nFilledLen >> 8) & 0xFF;
                pkt.data[3] = out->nFilledLen & 0xFF;

                if (out->nFlags & OMX_BUFFERFLAG_SYNCFRAME) {
                    pkt.flags |= AV_PKT_FLAG_KEY;
                }

                av_write_frame(m_mux_ctx, &pkt);
                if (pkt.data != out->pBuffer) {
                    free(pkt.data);
                }
            }
            out->nFilledLen = 0;
        }
    }
    return ret;
}

/**
 * Enqueue video to be encoded.
 * @param [in] mat The mat to be encoded.
 * @return true iff enqueued.
 */
bool OmxCvImpl::process(const cv::Mat &mat) {
    cv::Mat ours = mat.clone();

    auto now = steady_clock::now();
    std::unique_lock<std::mutex> lock(m_input_mutex);

    if (m_frame_count++ == 0) {
        m_frame_start = now;
    }

    if (m_input_queue.size() > 5) {
        printf("Queue too large; dropping frames!\n");
        m_input_queue.pop_front();
    }

    m_input_queue.push_back(std::pair<cv::Mat, int64_t>(ours, duration_cast<milliseconds>(now-m_frame_start).count()));
    lock.unlock();
    m_input_signaller.notify_one();

    return true;
}

/**
 * Constructor for our wrapper.
 * @param [in] name The file to save to.
 * @param [in] width The video width.
 * @param [in] height The video height.
 * @param [in] bitrate The bitrate, in Kbps.
 * @param [in] fpsnum The FPS numerator.
 * @param [in] fpsden The FPS denominator.
 */
OmxCv::OmxCv(const char *name, int width, int height, int bitrate, int fpsnum, int fpsden) {
    m_impl = new OmxCvImpl(name, width, height, bitrate, fpsnum, fpsden);
}

/**
 * Wrapper destructor.
 */
OmxCv::~OmxCv() {
    delete m_impl;
}

/**
 * Encode image.
 * @param [in] in Image to be encoded.
 */
void OmxCv::Encode(const cv::Mat &in) {
    m_impl->process(in);
}
