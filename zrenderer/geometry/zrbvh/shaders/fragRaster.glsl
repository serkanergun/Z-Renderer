#version 150
in float Depth;
out vec4 fragColor;

void main(void)
{
    fragColor = vec4(Depth, Depth, Depth, 1);
}
