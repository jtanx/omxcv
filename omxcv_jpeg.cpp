/**
 * @file omxcv_jpeg.cpp
 * @brief OpenMAX JPEG encoder for OpenCV.
 */

#include "omxcv.h"
#include "omxcv-impl.h"
#include <vector>

using namespace omxcv;

/**
 * Constructor.
 * @param [in] width The width of the image to encode.
 * @param [in] height The height of the image to encode.
 * @param [in] quality The JPEG quality factor (1-100). 100 is best quality.
 * @throws std::invalid_argument on error.
 */
OmxCvJpegImpl::OmxCvJpegImpl(int width, int height, int quality)
: m_width(width)
, m_height(height)
, m_stride(((width + 31) & ~31) * 3)
, m_quality(quality)
, m_stop{false}
{
    int ret;
    bcm_host_init();

    //Initialise OpenMAX and the IL client.
    CHECKED(OMX_Init() != OMX_ErrorNone, "OMX_Init failed.");
    m_ilclient = ilclient_init();
    CHECKED(m_ilclient == NULL, "ILClient initialisation failed.");

    ret = ilclient_create_component(m_ilclient, &m_encoder_component,
            (char*)"image_encode",
            (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS |
            ILCLIENT_ENABLE_INPUT_BUFFERS | ILCLIENT_ENABLE_OUTPUT_BUFFERS));
    CHECKED(ret != 0, "ILCient image_encode component creation failed.");

    //Set input definition to the encoder
    OMX_PARAM_PORTDEFINITIONTYPE def = {0};
    def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    def.nVersion.nVersion = OMX_VERSION;
    def.nPortIndex = OMX_JPEG_PORT_IN;
    ret = OMX_GetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamPortDefinition, &def);
    CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed for encode port in.");

    //We allocate 3 input buffers.
    def.nBufferCountActual = 3;
    def.format.image.nFrameWidth = m_width;
    def.format.image.nFrameHeight = m_height;
    //16 byte alignment. I don't know if these also hold for image encoding.
    def.format.image.nSliceHeight = (m_height + 15) & ~15;
    //Must be manually defined to ensure sufficient size if stride needs to be rounded up to multiple of 32.
    def.nBufferSize = def.format.image.nStride * def.format.image.nSliceHeight;
    def.format.image.nStride = m_stride;
    def.format.image.bFlagErrorConcealment = OMX_FALSE;
    def.format.image.eColorFormat =  OMX_COLOR_Format24bitBGR888; //OMX_COLOR_Format32bitABGR8888;//OMX_COLOR_FormatYUV420PackedPlanar;

    ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamPortDefinition, &def);
    CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed for input format definition.");

    //Set the output format of the encoder
    OMX_IMAGE_PARAM_PORTFORMATTYPE format = {0};
    format.nSize = sizeof(OMX_IMAGE_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = OMX_JPEG_PORT_OUT;
    format.eCompressionFormat = OMX_IMAGE_CodingJPEG;

    ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamImagePortFormat, &format);
    CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed for setting encoder output format.");

    //Set the encoder quality
    OMX_IMAGE_PARAM_QFACTORTYPE qfactor = {0};
    qfactor.nSize = sizeof(OMX_IMAGE_PARAM_QFACTORTYPE);
    qfactor.nVersion.nVersion = OMX_VERSION;
    qfactor.nPortIndex = OMX_JPEG_PORT_OUT;
    qfactor.nQFactor = m_quality;

    ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
            OMX_IndexParamQFactor, &qfactor);
    CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed for setting encoder quality.");

    ret = ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    CHECKED(ret != 0, "ILClient failed to change encoder to idle state.");
    ret = ilclient_enable_port_buffers(m_encoder_component, OMX_JPEG_PORT_IN, NULL, NULL, NULL);
    CHECKED(ret != 0, "ILClient failed to enable input buffers.");
    ret = ilclient_enable_port_buffers(m_encoder_component, OMX_JPEG_PORT_OUT, NULL, NULL, NULL);
    CHECKED(ret != 0, "ILClient failed to enable output buffers.");
    ret = ilclient_change_component_state(m_encoder_component, OMX_StateExecuting);
    CHECKED(ret != 0, "ILClient failed to change encoder to executing stage.");

    //Start the worker thread for dumping the encoded data
    m_input_worker = std::thread(&OmxCvJpegImpl::input_worker, this);
}

/**
 * Destructor.
 */
OmxCvJpegImpl::~OmxCvJpegImpl() {
    m_stop = true;
    m_input_signaller.notify_one();
    m_input_worker.join();

    //Teardown similar to hello_encode
    ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    ilclient_disable_port_buffers(m_encoder_component, OMX_JPEG_PORT_IN, NULL, NULL, NULL);
    ilclient_disable_port_buffers(m_encoder_component, OMX_JPEG_PORT_OUT, NULL, NULL, NULL);

    //ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
    ilclient_change_component_state(m_encoder_component, OMX_StateLoaded);

    COMPONENT_T *list[] = {m_encoder_component, NULL};
    ilclient_cleanup_components(list);
    ilclient_destroy(m_ilclient);
}

/**
 * Worker thread. Encodes the images.
 */
void OmxCvJpegImpl::input_worker() {
    std::unique_lock<std::mutex> lock(m_input_mutex);
    OMX_BUFFERHEADERTYPE *out = ilclient_get_output_buffer(m_encoder_component, OMX_JPEG_PORT_OUT, 1);

    while (true) {
        m_input_signaller.wait(lock, [this]{return m_stop || m_input_queue.size() > 0;});
        if (m_stop) {
            if (m_input_queue.size() > 0) {
                printf("Stop acknowledged but need to flush the input buffer (%d)...\n", m_input_queue.size());
            } else {
                break;
            }
        }

        std::pair<OMX_BUFFERHEADERTYPE *, std::string> frame = m_input_queue.front();
        m_input_queue.pop_front();
        lock.unlock();

        FILE *fp = fopen(frame.second.c_str(), "wb");
        if (!fp) {
            perror(frame.second.c_str());
        }

        OMX_EmptyThisBuffer(ILC_GET_HANDLE(m_encoder_component), frame.first);
        do {
            OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component), out);
            out = ilclient_get_output_buffer(m_encoder_component, OMX_JPEG_PORT_OUT, 1);
            if (fp) {
                fwrite(out->pBuffer, 1, out->nFilledLen, fp);
            }
        } while (!(out->nFlags & OMX_BUFFERFLAG_ENDOFFRAME));

        if (fp) {
            fclose(fp);
        }
        lock.lock();
    }

    //Needed because we call ilclient_get_output_buffer last.
    //Otherwise ilclient waits forever for the buffer to be filled.
    OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component), out);
}

/**
 * Process a frame.
 * @param [in] filename The filename to save to.
 * @param [in] mat The image data to save.
 * @return true iff the image will be saved. Will return false if there's no
 *         free input buffer.
 */
bool OmxCvJpegImpl::process(const char *filename, const cv::Mat &mat) {
    //static const std::vector<int> saveparams = {CV_IMWRITE_JPEG_QUALITY, 75};
    //cv::imwrite(filename, mat, saveparams);
    OMX_BUFFERHEADERTYPE *in = ilclient_get_input_buffer(
        m_encoder_component, OMX_JPEG_PORT_IN, 0);
    if (in == NULL) { //No free buffer.
        return false;
    }

    assert(mat.cols == m_width && mat.rows == m_height);
    BGR2RGB(mat, in->pBuffer, m_stride);
    in->nFilledLen = in->nAllocLen;

    std::unique_lock<std::mutex> lock(m_input_mutex);
    m_input_queue.push_back(std::pair<OMX_BUFFERHEADERTYPE *, std::string>(
        in, std::string(filename)));
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
OmxCvJpeg::OmxCvJpeg(int width, int height, int quality)
: m_width(width)
, m_height(height)
, m_quality(quality)
{
    m_impl = new OmxCvJpegImpl(width, height, quality);
}

/**
 * Wrapper destructor.
 */
OmxCvJpeg::~OmxCvJpeg() {
    delete m_impl;
}

/**
 * Encode image.
 * @param [in] filename The path to save the image to.
 * @param [in] in Image to be encoded. If the width and height of this
 *                image does not match what was set in the constructor,
 *                this function will fallback to using OpenCV's imwrite.
 * @param [in] fallback If set to true and there is no available buffer for
 * encoding, fallback to using OpenCV to write out the image.
 * @return true iff the file was encoded.
 */
bool OmxCvJpeg::Encode(const char *filename, const cv::Mat &in, bool fallback) {
    if (in.cols != m_width || in.rows != m_height || in.type() != CV_8UC3) {
        std::vector<int> params {cv::IMWRITE_JPEG_QUALITY, m_quality};
        return cv::imwrite(filename, in, params);
    }

    bool ret = m_impl->process(filename, in);
    if (!ret && fallback) {
        std::vector<int> params {cv::IMWRITE_JPEG_QUALITY, m_quality};
        return cv::imwrite(filename, in, params);
    }
    return ret;
}

