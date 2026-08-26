#include <cave-traversal-tool/OpenGL/Buffer.h>

// clang-format off
#include <glad/glad.h>
// clang-format on

#include <cstdio>

uint32_t opengl_buffer_create_buffer(const uint64_t size, const uint32_t flags, const void* data)
{
    uint32_t buffer_id = 0;

    glCreateBuffers(1, &buffer_id);
    glNamedBufferStorage(buffer_id, size, data, flags);

    return buffer_id;
}