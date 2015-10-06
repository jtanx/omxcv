/**
 * @file image_gpu.cpp
 * @brief GPU processing image library
 */

#include "image_gpu.h"
#include <assert.h>
#include <error.h>

#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <sstream>
#include <stdexcept>
#include <cstring>
#include <chrono>

using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

#define TIMEDIFF(start) (duration_cast<milliseconds>(steady_clock::now() - start).count())

#define check() assert(glGetError() == 0)

using namespace picopter;

void picopter::DisplayShit(int width, int height, void *data) {
    EGLBoolean result;
    EGLint num_config;
    
    static const EGLint attribute_list[] =
	{
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
		EGL_NONE
	};

	static const EGLint context_attributes[] = 
	{
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};
	EGLConfig config;
    
    //Get a display
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    assert(display!=EGL_NO_DISPLAY);
	check();
    
    // initialize the EGL display connection
	result = eglInitialize(display, NULL, NULL);
	assert(EGL_FALSE != result);
	check();
    
    // get an appropriate EGL frame buffer configuration
	result = eglChooseConfig(display, attribute_list, &config, 1, &num_config);
	assert(EGL_FALSE != result);
	check();

	// get an appropriate EGL frame buffer configuration
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(EGL_FALSE != result);
	check();

	// create an EGL rendering context
	EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attributes);
	assert(context!=EGL_NO_CONTEXT);
	check();
    
    // Create an offscreen rendering surface
    static const EGLint rendering_attributes[] =
	{
        EGL_WIDTH, width,
        EGL_HEIGHT, height,
        EGL_NONE
    };
    EGLSurface surface = eglCreatePbufferSurface(display, config, rendering_attributes);
    assert(surface != EGL_NO_SURFACE);
    check();
    
    //Bind the context to the current thread
    result = eglMakeCurrent(display, surface, surface, context);
    assert(result);
    check();

    picopter::GLProgram program("simplevertshader.glsl", "simplefragshader.glsl");
    picopter::GLTexture texture(width, height, GL_RGB);

    auto start = steady_clock::now();
    texture.SetData(data);
    printf("\nLOAD TIME: %d ms\n", (int)TIMEDIFF(start));

    //xyzw
    static const GLfloat quad_vertex_positions[] = {
        0.0f, 0.0f, 1.0f, 1.0f,
        1.0f, 0.0f, 1.0f, 1.0f,
        0.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f
    };

    GLuint buffer;
    glGenBuffers(1, &buffer);
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertex_positions), quad_vertex_positions, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    start = steady_clock::now();
    DoMoreShit(display, surface, buffer, program, texture);
    printf("RENDER TIME: %d ms\n", (int)TIMEDIFF(start));
    
}

void picopter::DoMoreShit(EGLDisplay display, EGLSurface surface, GLuint buffer, picopter::GLProgram &program, picopter::GLTexture &texture) {
    //Blank the display
    glBindFramebuffer(GL_FRAMEBUFFER, texture.GetFramebufferId());
    glViewport(0, 0, texture.GetWidth(), texture.GetHeight());
    check();
    glClear(GL_COLOR_BUFFER_BIT);
    check();
    
    glUseProgram(program);
    check();

    glUniform1i(glGetUniformLocation(program,"tex"), 0);
    glUniform4f(glGetUniformLocation(program, "threshLow"),0,167/255.0, 86/255.0,0);
    glUniform4f(glGetUniformLocation(program, "threshHigh"),255/255.0,255/255.0, 141/255.0,1);
    check();

    glBindBuffer(GL_ARRAY_BUFFER, buffer);   check();
    glBindTexture(GL_TEXTURE_2D, texture);  check();

    // Initialize the vertex position attribute from the vertex shader
    GLuint loc = glGetAttribLocation(program, "vPosition");
    glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);  check();
    glEnableVertexAttribArray(loc); check();

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4); check();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER,0);
    
    //glFinish(); check();
    //glFlush(); check();


    //RENDER
    eglSwapBuffers(display, surface);
    //check();

    //glFinish();
    
    /*void *out = malloc(3 * texture.GetWidth() * texture.GetHeight());
    texture.GetRenderedData(out);
    FILE *fp = fopen("TEMP.RGB", "wb");
    fwrite(out, 3 * texture.GetWidth() * texture.GetHeight(), 1, fp);
    fclose(fp);*/
}

namespace picopter {
GLProgram::GLProgram(const char *vertex_file, const char *fragment_file) {
    GLint status;
    m_program_id = glCreateProgram();

    m_vertex_id = LoadShader(GL_VERTEX_SHADER, vertex_file);
    m_fragment_id = LoadShader(GL_FRAGMENT_SHADER, fragment_file);
    glAttachShader(m_program_id, m_vertex_id);
    glAttachShader(m_program_id, m_fragment_id);
    
    glLinkProgram(m_program_id);
    glGetProgramiv(m_program_id, GL_LINK_STATUS, &status);
    if (!status) {
        GLint msg_len;
        char *msg;
        std::stringstream s;

        glGetProgramiv(m_program_id, GL_INFO_LOG_LENGTH, &msg_len);
        msg = new char[msg_len];
        glGetProgramInfoLog(m_program_id, msg_len, NULL, msg);

        s << "Failed to link shaders: " << msg;
        delete [] msg;
        throw std::invalid_argument(s.str());
    }
}

GLProgram::~GLProgram() {
    glDeleteShader(m_fragment_id);
    glDeleteShader(m_vertex_id);
    glDeleteProgram(m_program_id);
}

GLuint GLProgram::GetId() {
    return m_program_id;
}

GLuint GLProgram::LoadShader(GLenum shader_type, const char *source_file) {
    GLint status;
    GLuint shader_id;
    char *shader_source = ReadFile(source_file);

    if (!shader_source) {
        std::stringstream s;
        const char *error = std::strerror(errno);
        s << "Could not load " << source_file << ": " << error;
        throw std::invalid_argument(s.str());
    }

    shader_id = glCreateShader(shader_type);
    glShaderSource(shader_id, 1, (const GLchar**)&shader_source, NULL);
    glCompileShader(shader_id);
    delete [] shader_source;

    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &status);
    if (!status) {
        GLint msg_len;
        char *msg;
        std::stringstream s;

        glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &msg_len);
        msg = new char[msg_len];
        glGetShaderInfoLog(shader_id, msg_len, NULL, msg);

        s << "Failed to compile " << source_file << ": " << msg;
        delete [] msg;
        throw std::invalid_argument(s.str());
    }

    return shader_id;
}

char* GLProgram::ReadFile(const char *file) {
    std::FILE *fp = std::fopen(file, "rb");
    char *ret = NULL;
    size_t length;

    if (fp) {
        std::fseek(fp, 0, SEEK_END);
        length = std::ftell(fp);
        std::fseek(fp, 0, SEEK_SET);

        ret = new char[length + 1];
        length = std::fread(ret, 1, length, fp);
        ret[length] = '\0';

        std::fclose(fp);
    }

    return ret;
}

GLTexture::GLTexture(GLsizei width, GLsizei height, GLint type)
: m_width(width), m_height(height), m_type(type)
{
    if (width % 4 || height % 4) {
        throw std::invalid_argument("Width/height is not a multiple of 4.");
    }

    //Allocate the texture buffer
    glGenTextures(1, &m_texture_id);
    glBindTexture(GL_TEXTURE_2D, m_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, type, width, height, 0, type, GL_UNSIGNED_BYTE, NULL);
    if (glGetError() != GL_NO_ERROR) {
        throw std::invalid_argument("glTexImage2D failed. Could not allocate texture buffer.");
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    //Allocate the frame buffer
    glGenFramebuffers(1, &m_framebuffer_id);
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer_id);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texture_id, 0);
    if (glGetError() != GL_NO_ERROR) {
        throw std::invalid_argument("glFramebufferTexture2D failed. Could not allocate framebuffer.");
    }
    glBindFramebuffer(GL_FRAMEBUFFER,0);
}

GLTexture::~GLTexture() {
    glDeleteFramebuffers(1, &m_framebuffer_id);
    glDeleteTextures(1, &m_texture_id);
}

GLsizei GLTexture::GetWidth() {
    return m_width;
}

GLsizei GLTexture::GetHeight() {
    return m_height;
}

void GLTexture::SetData(void *data) {
    glBindTexture(GL_TEXTURE_2D, m_texture_id);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_width, m_height, m_type, GL_UNSIGNED_BYTE, data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void GLTexture::GetRenderedData(void *buffer) {
    glBindFramebuffer(GL_FRAMEBUFFER,m_framebuffer_id);
    glReadPixels(0, 0, m_width, m_height, m_type, GL_UNSIGNED_BYTE, buffer);
    glBindFramebuffer(GL_FRAMEBUFFER,0);
}

GLuint GLTexture::GetTextureId() {
    return m_texture_id;
}

GLuint GLTexture::GetFramebufferId() {
    return m_framebuffer_id;
}

} //namespace picopter
