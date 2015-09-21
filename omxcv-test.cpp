/**
 * @file omxcv-test.cpp
 * @brief Simple testing application for omxcv.
 */
#include "omxcv.h"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>

#define TIMEDIFF(start) (duration_cast<milliseconds>(steady_clock::now() - start).count())

using omxcv::OmxCv;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

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
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    capture.set(CV_CAP_PROP_FPS, 30);

    OmxCv e((const char*)"save.mkv", (int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT), 4000);

    auto totstart = steady_clock::now();
    cv::Mat image;
    for(int i = 0; i < framecount; i++) {
        capture >> image;
        //auto start = steady_clock::now();
        e.Encode(image);
        //printf("Processed frame %d (%d ms)\r", i+1, (int)TIMEDIFF(start));
        //fflush(stdout);
    }

    //FILE *fp = fopen("Orig.rgb", "wb");
    //fwrite(image.data, 3 * image.cols * image.rows, 1, fp);
    //fclose(fp);
    //picopter::DisplayShit(image.cols, image.rows, image.data);

    printf("Average FPS: %.2f\n", (framecount * 1000) / (float)TIMEDIFF(totstart));
    printf("DEPTH: %d, WIDTH: %d, HEIGHT: %d, IW: %d\n", image.depth(), image.cols, image.rows, static_cast<int>(image.step));
    sleep_for(milliseconds(300));

    //bcm_host_deinit();
}
