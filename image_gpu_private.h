/**
 * @file image_gpu_private.h
 * @brief Private GPU header. GPU functions for image processing.
 */

#ifndef _PICOPTERX_IMAGE_GPU_PRIVATE_H
#define _PICOPTERX_IMAGE_GPU_PRIVATE_H

#include <GLES2/gl2.h>
#include <EGL/egl.h>

namespace picopter {
    class GLProgram {
        public:
            GLProgram(const char *vertex_file, const char *fragment_file);
            virtual ~GLProgram();

            GLuint GetId();
            operator GLuint() { return m_program_id; };
        private:
            GLuint m_vertex_id, m_fragment_id, m_program_id;

            GLuint LoadShader(GLenum shader_type, const char *source_file);
            char* ReadFile(const char *file);
    };

    class GLTexture {
        public:
            GLTexture(GLsizei width, GLsizei height, GLint type);
            virtual ~GLTexture();

            GLsizei GetWidth();
            GLsizei GetHeight();

            void SetData(void *data);
            void GetRenderedData(void *buffer);
            GLuint GetTextureId();
            GLuint GetFramebufferId();
            operator GLuint() { return m_texture_id; };
        private:
            GLsizei m_width, m_height;
            GLint m_type;
            GLuint m_texture_id, m_framebuffer_id;
    };

    extern void DoMoreShit(EGLDisplay display, EGLSurface surface, GLuint buffer, picopter::GLProgram &program, picopter::GLTexture &texture);
    extern void DisplayShit(int width, int height, void *data);
}

#endif // _PICOPTERX_IMAGE_GPU_PRIVATE_H