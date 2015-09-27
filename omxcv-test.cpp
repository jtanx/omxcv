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

#define TIMEDIFF(start) (duration_cast<microseconds>(steady_clock::now() - start).count())

using omxcv::OmxCv;
using omxcv::OmxCvJpeg;
using std::this_thread::sleep_for;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

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

    //OmxCv e((const char*)"save.mkv", (int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT), 4000);
    OmxCvJpeg j((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    
    auto totstart = steady_clock::now();
    cv::Mat image;
    //FILE *fp = fopen("log.txt", "w");
    
    for(int i = 0; i < framecount; i++) {
        capture >> image;
        auto start = steady_clock::now();
        if (j.Encode("save.jpg", image)) {
        //if (e.Encode(image)) {
            printf("Processed frame %4d (%4d ms)\r", i+1, (int)TIMEDIFF(start)/1000);
            fflush(stdout);
            //fprintf(fp, "%d\n", (int)TIMEDIFF(start));
            processed++;
        }
    }
    //fclose(fp);

    //FILE *fp = fopen("Orig.rgb", "wb");
    //fwrite(image.data, 3 * image.cols * image.rows, 1, fp);
    //fclose(fp);
    //picopter::DisplayShit(image.cols, image.rows, image.data);

    printf("Average FPS: %.2f\n", (processed * 1000000) / (float)TIMEDIFF(totstart));
    printf("Processed: %d, Dropped: %d\n", processed, framecount-processed);
    printf("DEPTH: %d, WIDTH: %d, HEIGHT: %d, IW: %d\n", image.depth(), image.cols, image.rows, static_cast<int>(image.step));
    sleep_for(milliseconds(300));

    //bcm_host_deinit();
}
