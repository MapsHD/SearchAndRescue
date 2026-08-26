#version 460 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Color;

layout(location = 0) uniform mat4 u_MVP = mat4(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    shared_data.color = 0.5 * in_Color + 0.5;

    gl_Position = u_MVP * vec4(in_Position, 1.0f);
}