/**
 * @file omxcv.cpp
 * @brief Header file for the library
 */

#ifndef __OMXCV_H
#define __OMXCV_H

#include <opencv2/opencv.hpp>

namespace omxcv {
    /* Forward declaration of our implementation. */
    class OmxCvImpl;

    /**
     * Real-time OpenMAX H.264 encoder for the Raspberry Pi/OpenCV.
     */
    class OmxCv {
        public:
            OmxCv(const char *name, int width, int height, int bitrate=3000, int fpsnum=25, int fpsden=1);
            void Encode(const cv::Mat &in);
            virtual ~OmxCv();
        private:
            OmxCvImpl *m_impl;
    };
}

#endif /* _OMXCV_H */