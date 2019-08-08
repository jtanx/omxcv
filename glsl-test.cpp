/**
 * @file glsl-test.cpp
 * @brief Simple testing application for GLSL.
 */

#include "image_gpu.h"

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#define TIMEDIFF(start) (duration_cast<microseconds>(steady_clock::now() - start).count())

using std::this_thread::sleep_for;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

using namespace picopter;

int main(int argc, char *argv[]) {
    cv::VideoCapture capture(-1);

    int width = 640, height = 480, framecount = 200, processed = 0;

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
#if (CV_MAJOR_VERSION < 3)
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    capture.set(CV_CAP_PROP_FPS, 30);
#else
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    capture.set(cv::CAP_PROP_FPS, 30);
#endif // (CV_MAJOR_VERSION < 3)
    
    auto totstart = steady_clock::now();
    cv::Mat image, out;
    //FILE *fp = fopen("log.txt", "w");
    
#if (CV_MAJOR_VERSION < 3)
    GLThreshold t(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
#else
    GLThreshold t(capture.get(cv::CAP_PROP_FRAME_WIDTH), capture.get(cv::CAP_PROP_FRAME_HEIGHT));
#endif // (CV_MAJOR_VERSION < 3)
    
    for(int i = 0; i < framecount; i++) {
        capture >> image;
        auto start = steady_clock::now();
        t.Threshold(image, out);
        printf("Processed frame %4d (%4d ms)\r", i+1, (int)TIMEDIFF(start)/1000);
        fflush(stdout);
    }
}