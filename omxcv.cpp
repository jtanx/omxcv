/**
 * @file omxcv.cpp
 * @brief Routines to perform hardware accelerated H.264 encoding on the RPi.
 */

#include "omxcv-impl.h"
using namespace omxcv;

using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>

#define TIMEDIFF(start) (duration_cast<milliseconds>(steady_clock::now() - start).count())

bool OmxCvImpl::lav_init(const char *filename, int width, int height, int bitrate, int fpsnum, int fpsden) {
    int ret;

    //Allocate an AVFrame for conversion to YUV colurspace
    m_omx_in = OMXCV_AV_FRAME_ALLOC();
    if (m_omx_in == NULL) {
        return false;
    }


    av_register_all();

    //Do we need to free this?
    AVOutputFormat *fmt = av_guess_format(NULL, filename, NULL);
    if (fmt == NULL) {
        return false;
    }

    m_mux_ctx = avformat_alloc_context();
    if (m_mux_ctx == NULL) {
        return false;
    }
    m_mux_ctx->debug = 1;
    m_mux_ctx->start_time_realtime = AV_NOPTS_VALUE;
    m_mux_ctx->start_time = AV_NOPTS_VALUE;
    m_mux_ctx->duration = 0;
    m_mux_ctx->bit_rate = 0;
    m_mux_ctx->oformat = fmt;
    snprintf(m_mux_ctx->filename, sizeof(m_mux_ctx->filename), "%s", filename);

    m_video_stream = avformat_new_stream(m_mux_ctx, NULL);
    if (m_video_stream == NULL) {
        return false;
    }

    m_video_stream->codec->width = width;
    m_video_stream->codec->height = height;
    m_video_stream->codec->codec_id = CODEC_ID_H264;
    m_video_stream->codec->codec_type = AVMEDIA_TYPE_VIDEO;
    m_video_stream->codec->bit_rate = bitrate;
    m_video_stream->codec->profile = FF_PROFILE_H264_HIGH;
    m_video_stream->codec->level = 41;
    m_video_stream->codec->time_base.num = fpsnum;
    m_video_stream->codec->time_base.den = fpsden;
    
    m_video_stream->time_base.num = fpsnum;
    m_video_stream->time_base.den = fpsden;
    
    m_video_stream->r_frame_rate.num = fpsnum;
    m_video_stream->r_frame_rate.den = fpsden;
    
    m_video_stream->start_time = AV_NOPTS_VALUE;

    m_video_stream->codec->sample_aspect_ratio.num = m_video_stream->sample_aspect_ratio.num;
    m_video_stream->codec->sample_aspect_ratio.den = m_video_stream->sample_aspect_ratio.den;

    if (avio_open(&m_mux_ctx->pb, filename, AVIO_FLAG_WRITE) < 0) {
        return 0;
    }

    if (m_mux_ctx->oformat->flags & AVFMT_GLOBALHEADER) {
        m_mux_ctx->streams[0]->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;
    }

    avformat_write_header(m_mux_ctx, NULL);
    //Print info about this format
    av_dump_format(m_mux_ctx, 0, filename, 1);

    return true;
}

OmxCvImpl::OmxCvImpl(const char *name, int width, int height, int bitrate, int fpsnum, int fpsden)
: m_sws_ctx(NULL)
, m_stop{false}
{
    bcm_host_init();
    OMX_Init();
    //Imposed restrictions due to slice height and stride width 
    m_width = ((width + 31) / 32) * 32;
    m_height = ((height + 15) / 16) * 16;
    if (m_width != width || m_height != height) {
        printf("Warning: Resize necessary due to width/height not being a multiple of 32/16, expect it to be slow!\n");
    }
    m_ilclient = ilclient_init();
    ilclient_create_component(m_ilclient, &m_encoder_component, (char*)"video_encode", 
                          (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS | 
                          ILCLIENT_ENABLE_INPUT_BUFFERS | 
                          ILCLIENT_ENABLE_OUTPUT_BUFFERS));
    
    //Set input definition to the encoder                      
    OMX_PARAM_PORTDEFINITIONTYPE def = {0};
    def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    def.nVersion.nVersion = OMX_VERSION;
    def.nPortIndex = OMX_ENCODE_PORT_IN;
    OMX_GetParameter(ILC_GET_HANDLE(m_encoder_component),
                     OMX_IndexParamPortDefinition, &def);
                     
    def.format.video.nFrameWidth = m_width;
    def.format.video.nFrameHeight = m_height;
    def.format.video.xFramerate = 25 << 16;
    //Must be a multiple of 16
    def.format.video.nSliceHeight = m_height;
    //Must be a multiple of 32
    def.format.video.nStride = m_width;
    def.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;

    OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
                     OMX_IndexParamPortDefinition, &def);

    //Set the output format of the encoder
    OMX_VIDEO_PARAM_PORTFORMATTYPE format = {0};
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = OMX_ENCODE_PORT_OUT;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;

    OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
                     OMX_IndexParamVideoPortFormat, &format);

    //Set the encoding bitrate
    OMX_VIDEO_PARAM_BITRATETYPE bitrate_type = {0};
    bitrate_type.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    bitrate_type.nVersion.nVersion = OMX_VERSION;
    bitrate_type.eControlRate = OMX_Video_ControlRateVariable;
    bitrate_type.nTargetBitrate = bitrate * 1000;
    bitrate_type.nPortIndex = OMX_ENCODE_PORT_OUT;
    OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
                     OMX_IndexParamVideoBitrate, &bitrate_type);


    ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    ilclient_enable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_IN, NULL, NULL, NULL);
    ilclient_enable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_OUT, NULL, NULL, NULL);
    ilclient_change_component_state(m_encoder_component, OMX_StateExecuting);

    if (fpsden <= 0 || fpsnum <= 0) {
        fpsden = 1;
        fpsnum = 25;
    }
    m_fpsnum = fpsnum;
    m_fpsden = fpsden;
    //Initialise the scaler and output file
    lav_init(name, m_width, m_height, bitrate, fpsnum, fpsden);

    m_frame_count = 0;
    m_timecodes = fopen("timecodes.txt", "w");
    fprintf(m_timecodes, "# timecode format v2\n");

    //Start the worker thread for dumping the encoded data
    m_worker = std::thread(&OmxCvImpl::worker, this);
}

OmxCvImpl::~OmxCvImpl() {
    m_stop = true;
    m_worker.join();

    //Teardown similar to hello_encode
    ilclient_disable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_IN, NULL, NULL, NULL);
    ilclient_disable_port_buffers(m_encoder_component, OMX_ENCODE_PORT_OUT, NULL, NULL, NULL);

    ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    ilclient_change_component_state(m_encoder_component, OMX_StateLoaded);

    COMPONENT_T *list[] = {m_encoder_component, NULL};
    ilclient_cleanup_components(list);
    ilclient_destroy(m_ilclient);

    //Close the output file
    avio_close(m_mux_ctx->pb);
    avformat_free_context(m_mux_ctx);
    fclose(m_timecodes);
}

void OmxCvImpl::worker() {
    while (!m_stop) {
        static int i = 0;

        OMX_BUFFERHEADERTYPE *out = ilclient_get_output_buffer(m_encoder_component, OMX_ENCODE_PORT_OUT, 0);

        if (out != NULL) {
            if (out->nFilledLen > 0) {
                //Check for an SPS/PPS header
                static const uint8_t header_sig[] = {0,0,0,1};
                if (out->nFilledLen > 5 && !memcmp(out->pBuffer, header_sig, 4)) {
                    if (out->pBuffer[4] == 0x67) {
                        //SPS - DO NOTHING RIGHT NOW
                        out->nFilledLen = 0;
                        continue;
                    } else if (out->pBuffer[4] == 0x8) {
                        //PPS
                        out->nFilledLen = 0;
                        continue;
                    }
                }

                AVPacket pkt;
                
                //This doesn't seem to give anything useful
                OMX_TICKS tick = out->nTimeStamp;

                printf("TICK: %lld\n", tick);
                av_init_packet(&pkt);
                pkt.stream_index = m_video_stream->index;
                pkt.data= out->pBuffer;
                pkt.size= out->nFilledLen;

                if (out->nFlags & OMX_BUFFERFLAG_SYNCFRAME) {
                    pkt.flags |= AV_PKT_FLAG_KEY;
                }

                /*
                pkt.pts = i++;
                pkt.dts = AV_NOPTS_VALUE;
                */
                
                av_write_frame(m_mux_ctx, &pkt);
                out->nFilledLen = 0;
            }
            OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component), out);
        } else {
            //printf("Zero buffer received; sleeping...\n");
            sleep_for(milliseconds(300));
        }
    }
}

bool OmxCvImpl::process(const cv::Mat &mat) {
    int ret;
    
    //Well, we could just write out the raw H.264 stream and get mkvmerge to mux
    //it with these timecodes. So much simpler than libav...
    if (m_frame_count++ == 0) {
        fprintf(m_timecodes, "0\n");
        m_frame_start = steady_clock::now();
    } else {
        int diff = TIMEDIFF(m_frame_start);
        fprintf(m_timecodes, "%d\n", diff);
    }

    auto start = steady_clock::now();
    OMX_BUFFERHEADERTYPE *in = ilclient_get_input_buffer(m_encoder_component, OMX_ENCODE_PORT_IN, 1);
    if (in == NULL) {
        printf("NO INPUT BUFFER");
        return false;
    }
    int td = (int)TIMEDIFF(start);
    if (td > 0) {
        printf("Time to get buffer (ms): %d\n",td);
    }
    //Recheck the context
    m_sws_ctx = sws_getCachedContext(m_sws_ctx, mat.cols, mat.rows,
                                     PIX_FMT_BGR24, m_width, m_height,
                                     PIX_FMT_YUV420P, SWS_BICUBIC,
                                     NULL, NULL, NULL);

    //Set the encoder input buffer as the output
    avpicture_fill((AVPicture*)m_omx_in,
                                    in->pBuffer,
                                    PIX_FMT_YUV420P,
                                    m_width, m_height);
    in->nFilledLen = in->nAllocLen;
    printf("YUV420P linesize: %d, %d, %d\n", m_omx_in->linesize[0], m_omx_in->linesize[1], m_omx_in->linesize[2]);

    //Perform the transform
    int linesize[4] = {mat.step, 0, 0, 0};
    ret = sws_scale(m_sws_ctx, (uint8_t **) &(mat.data),
                    linesize, 0, mat.rows, m_omx_in->data, m_omx_in->linesize);
    if (ret < 0) {
        return false;
    }

    OMX_EmptyThisBuffer(ILC_GET_HANDLE(m_encoder_component), in);
    return true;
}

int main(int argc, char *argv[]) {
    cv::VideoCapture capture(-1);
    int width = 640, height = 480, framecount = 200;

    if (argc >= 3) {
        width = atoi(argv[1]);
        height = atoi(argv[2]);
    }

    if (argc >= 4) {
        framecount = atoi(argv[3]);
    }
    
    //Open the camera (testing only)
    if (!capture.isOpened()) {
        printf("Cannot open camera\n");
        return 1;
    }
    //We can try to set these, but the camera may ignore this anyway...
    capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    capture.set(CV_CAP_PROP_FPS, 30);
    
    OmxCvImpl e((const char*)"save.mkv", (int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT), 4000);
    cv::Mat image;
    auto totstart = steady_clock::now();
    for(int i = 0; i < framecount; i++) {
        capture >> image;
        auto start = steady_clock::now();
        e.process(image);
        printf("Processed frame %d (%d ms)\n", i+1, (int)TIMEDIFF(start));
    }
    
    printf("Average FPS: %.2f\n", (framecount * 1000) / (float)TIMEDIFF(totstart));
    printf("DEPTH: %d, WIDTH: %d, HEIGHT: %d, IW: %d\n", image.depth(), image.cols, image.rows, static_cast<int>(image.step));
    sleep_for(milliseconds(300));
    
    //bcm_host_deinit();
}
