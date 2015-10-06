/**
 * @file image_gpu.cpp
 * @brief GPU processing image library
 */

#include "image_gpu.h"
#include "image_gpu_private.h"
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

#define CHECKED(c, v) if ((c)) throw std::invalid_argument(v)
#define GLCHECKED(c, v) if ((c) || glGetError() != 0) throw std::invalid_argument(v)
#define TIMEDIFF(start) (duration_cast<milliseconds>(steady_clock::now() - start).count())

#define check() assert(glGetError() == 0)

using namespace picopter;

GLThreshold::GLThreshold(Options *opts, int width, int height)
: m_width(width)
, m_height(height)
, m_thresh_min{}
, m_thresh_max{}
{
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
    m_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    GLCHECKED(m_display == EGL_NO_DISPLAY, "Cannot get EGL display.");
    
    //Initialise the EGL display connection
	result = eglInitialize(m_display, NULL, NULL);
    GLCHECKED(result == EGL_FALSE, "Cannot initialise display connection.");
    
    //Get an appropriate EGL frame buffer configuration
	result = eglChooseConfig(m_display, attribute_list, &config, 1, &num_config);
    GLCHECKED(result == EGL_FALSE, "Cannot get buffer configuration.");

	//Bind to the right EGL API.
	result = eglBindAPI(EGL_OPENGL_ES_API);
    GLCHECKED(result == EGL_FALSE, "Could not bind EGL API.");

	//Create an EGL rendering context
	EGLContext context = eglCreateContext(m_display, config, EGL_NO_CONTEXT, context_attributes);
    GLCHECKED(context == EGL_NO_CONTEXT, "Could not create EGL context.");
    
    //Create an offscreen rendering surface
    static const EGLint rendering_attributes[] =
	{
        EGL_WIDTH, width,
        EGL_HEIGHT, height,
        EGL_NONE
    };
    m_surface = eglCreatePbufferSurface(m_display, config, rendering_attributes);
    GLCHECKED(m_surface == EGL_NO_SURFACE, "Could not create PBuffer surface.");
    
    //Bind the context to the current thread
    result = eglMakeCurrent(m_display, m_surface, m_surface, context);
    GLCHECKED(result == EGL_FALSE, "Failed to bind context.");
    
    //xyzw
    static const GLfloat quad_vertex_positions[] = {
        0.0f, 0.0f, 1.0f, 1.0f,
        1.0f, 0.0f, 1.0f, 1.0f,
        0.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f
    };

    glGenBuffers(1, &m_quad_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, m_quad_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertex_positions), quad_vertex_positions, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //Setup the shaders and texture buffer.
    m_program = new GLProgram("simplevertshader.glsl", "simplefragshader.glsl");
    m_texture = new GLTexture(width, height, GL_RGB);
}

GLThreshold::GLThreshold(int width, int height)
: GLThreshold(NULL, width, height) {}

GLThreshold::~GLThreshold() {
    eglDestroySurface(m_display, m_surface);
    delete m_program;
    delete m_texture;
}

void GLThreshold::Threshold(const cv::Mat &in, cv::Mat &out) {
    //Load the data into a texture.
    m_texture->SetData(in.data);
    
    //Blank the display
    glBindFramebuffer(GL_FRAMEBUFFER, m_texture->GetFramebufferId());
    glViewport(0, 0, m_texture->GetWidth(), m_texture->GetHeight());
    check();
    glClear(GL_COLOR_BUFFER_BIT);
    check();
    
    glUseProgram(*m_program);
    check();

    //Load in the texture and thresholding parameters.
    glUniform1i(glGetUniformLocation(*m_program,"tex"), 0);
    glUniform4f(glGetUniformLocation(*m_program, "threshLow"),0,167/255.0, 86/255.0,0);
    glUniform4f(glGetUniformLocation(*m_program, "threshHigh"),255/255.0,255/255.0, 141/255.0,1);
    check();

    glBindBuffer(GL_ARRAY_BUFFER, m_quad_buffer);   check();
    glBindTexture(GL_TEXTURE_2D, *m_texture);  check();

    //Initialize the vertex position attribute from the vertex shader
    GLuint loc = glGetAttribLocation(*m_program, "vPosition");
    glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);  check();
    glEnableVertexAttribArray(loc); check();

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4); check();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER,0);
    
    //glFinish(); check();
    //glFlush(); check();

    //RENDER
    eglSwapBuffers(m_display, m_surface);
    //check();

    //glFinish();
    
    /*void *out = malloc(3 * m_texture->GetWidth() * m_texture->GetHeight());
    m_texture->GetRenderedData(out);
    FILE *fp = fopen("TEMP.RGB", "wb");
    fwrite(out, 3 * m_texture->GetWidth() * m_texture->GetHeight(), 1, fp);
    fclose(fp);*/
}

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
