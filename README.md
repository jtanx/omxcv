# omxcv
GPU assisted (hardware) H.264 encoder for OpenCV on the Raspberry Pi using OpenMAX.

Allows OpenCV Mat data to be encoded easily in real-time using the GPU encoder on the Raspberry Pi. On the Raspberry Pi 2, resolutions up to 640x480 can be saved in real-time (i.e. maintaining 30FPS).

omxcv maintains an internal clock, so the time at which the frame is sent to omxcv is when it will be displayed on playback. In addition, omxcv **will drop** frames if it detects that it is falling behind. 

Restrictions:
* Width should be mod8
* Input `Mat`s should be of type CV_8UC3 BGR images.
* Output format should either be MKV or MP4.
