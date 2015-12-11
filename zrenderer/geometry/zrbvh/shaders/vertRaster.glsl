#version 150

uniform mat3x4 projMatrix;

in vec3 vertPos;
out float Depth;

void main(void)
{
    vec3 pos    = vec4(vertPos, 1) * projMatrix;
    gl_Position = vec4(pos, 1);
    Depth       = pos.z;
}
