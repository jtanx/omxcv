varying vec2 tcoord;
uniform sampler2D tex;
uniform vec4 threshLow, threshHigh;

void main(void) 
{
    mat4 RGBtoYUV = mat4(0.257,  0.439, -0.148, 0.0,
              0.504, -0.368, -0.291, 0.0,
              0.098, -0.071,  0.439, 0.0,
              0.0625, 0.500,  0.500, 1.0 );

    //Y'CrCb
	vec4 yuv = RGBtoYUV * texture2D(tex,tcoord).bgra;
    bvec4 l = greaterThanEqual(yuv, threshLow);
    bvec4 h = lessThanEqual(yuv, threshHigh);
    
    if (all(l) && all(h)) {
        gl_FragColor = vec4(1,1,1,1);
    } else {
        gl_FragColor = vec4(0,0,0,1);
    }
}
