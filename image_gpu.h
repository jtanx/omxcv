/**
 * @file image_gpu.h
 * @brief GPU functions for image processing.
 */

#ifndef _PICOPTERX_IMAGE_GPU_H
#define _PICOPTERX_IMAGE_GPU_H

#include <opencv2/opencv.hpp>
#include <EGL/egl.h>
#include <GLES2/gl2.h>

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    /* Forward declaration of the GLProgram class */
    class GLProgram;
    /* Forward declaration of the GLTexture class */
    class GLTexture;
    
    typedef struct ThresholdSet {
        int x;
        int y;
        int z;
    } ThresholdSet;
    
    /**
     * Class to perform colour thresholding using OpenGL.
     */
    class GLThreshold {
            public:
                GLThreshold(Options *opts, int width, int height);
                GLThreshold(int width, int height);
                virtual ~GLThreshold();
                
                void SetThresholds(ThresholdSet min, ThresholdSet max);
                void Threshold(const cv::Mat &in, cv::Mat &out);
            private:
                int m_width, m_height;
                ThresholdSet m_thresh_min, m_thresh_max;
                GLProgram *m_program;
                GLTexture *m_texture;
                
                EGLDisplay m_display;
                EGLSurface m_surface;
                GLuint m_quad_buffer;
                
                /** Copy constructor (disabled) **/
                GLThreshold(const GLThreshold &other);
                /** Assignment operator (disabled) **/
                GLThreshold& operator= (const GLThreshold &other);
        };
    
}
    
#endif