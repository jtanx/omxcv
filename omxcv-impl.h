/**
 * @file omxcv-impl.cpp
 * @brief Actual implementation class
 */

#ifndef __OMXCV_IMPL_H
#define __OMXCV_IMPL_H

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <opencv2/opencv.hpp>

//Sigh
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/avutil.h>
#include <libavutil/mathematics.h>
#include <libavformat/avio.h>

//Sigh
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvariadic-macros"
#include <bcm_host.h>
#include <ilclient.h>
#pragma GCC diagnostic pop
}



//Determine what frame allocation routine to use
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
    #define OMXCV_AV_FRAME_ALLOC av_frame_alloc
    #define OMXCV_AV_FRAME_FREE av_frame_free
#else
    #define OMXCV_AV_FRAME_ALLOC avcodec_alloc_frame
    #define OMXCV_AV_FRAME_FREE avcodec_free_frame
#endif

#define OMX_ENCODE_PORT_IN  200
#define OMX_ENCODE_PORT_OUT 201

namespace omxcv {
    class OmxCvImpl {
        public:
            OmxCvImpl(const char *name, int width, int height, int bitrate, int fpsnum=-1, int fpsden=-1);
            virtual ~OmxCvImpl();
            
            bool process(const cv::Mat &mat);
        private:
            int m_width, m_height, m_fpsnum, m_fpsden;
            AVFrame *m_omx_in;
            SwsContext *m_sws_ctx;

            std::thread m_worker;
            std::atomic<bool> m_stop;
            
            /** The OpenMAX IL client **/
            ILCLIENT_T *m_ilclient;
            COMPONENT_T *m_encoder_component;

            /** Writine out stuff **/
            AVFormatContext *m_mux_ctx;
            AVStream *m_video_stream;

            FILE *m_timecodes;
            std::chrono::steady_clock::time_point m_frame_start;
            int m_frame_count;

            void worker();
            bool lav_init(const char *filename, int width, int height, int bitrate, int fpsnum, int fpsden);
    };
}

#endif
