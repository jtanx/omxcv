attribute vec4 vPosition;
varying vec2 tcoord;

void main(void)
{
    vec4 pos = vPosition;
    tcoord = pos.xy;
    pos.xy = pos.xy*vec2(2,2) + vec2(-1,-1);
    gl_Position = pos;
}
