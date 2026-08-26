#version 460 core

layout(location = 0) in vec3 in_Position;

layout(location = 0) uniform mat4 u_MVP   = mat4(1.0f);
layout(location = 1) uniform vec3 u_Color = vec3(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;

    gl_Position = u_MVP * vec4(in_Position, 1.0f);
}