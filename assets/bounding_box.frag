#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    out_Color = shared_data.color;
}