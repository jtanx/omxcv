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
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    capture.set(CV_CAP_PROP_FPS, 30);
    
    auto totstart = steady_clock::now();
    cv::Mat image;
    //FILE *fp = fopen("log.txt", "w");
    
    for(int i = 0; i < framecount; i++) {
        capture >> image;
        DisplayShit(image.cols, image.rows, image.data);
    }
}