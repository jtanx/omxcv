varying vec2 tcoord;
uniform sampler2D tex;
void main(void) 
{
    mat4 RGBtoYUV = mat4(0.257,  0.439, -0.148, 0.0,
              0.504, -0.368, -0.291, 0.0,
              0.098, -0.071,  0.439, 0.0,
              0.0625, 0.500,  0.500, 1.0 );
    gl_FragColor = RGBtoYUV * texture2D(tex,tcoord).bgra;

    //This is not in packed planar format.
}
