#include <Core/OpenGL/Buffer.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct Buffer::BufferIMPL
{
    uint32_t    id    = {};
    BufferUsage usage = {};
    size_t      size  = {};
};

static GLenum to_gl_usage(const BufferUsage usage)
{
    switch (usage)
    {
    case BufferUsage::Static:
        return GL_STATIC_DRAW;
    case BufferUsage::Dynamic:
        return GL_DYNAMIC_DRAW;
    }
    return GL_STATIC_DRAW;
}

Buffer::Buffer(const BufferUsage usage, const size_t size, const void* data)
{
    _impl = new BufferIMPL;

    _impl->id    = UINT32_MAX;
    _impl->usage = usage;
    _impl->size  = size;

    glGenBuffers(1, &_impl->id);
    glBindBuffer(GL_COPY_WRITE_BUFFER, _impl->id);
    glBufferData(GL_COPY_WRITE_BUFFER, static_cast<GLsizeiptr>(_impl->size), data, to_gl_usage(_impl->usage));
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

Buffer::~Buffer()
{
    glDeleteBuffers(1, &_impl->id);

    delete _impl;
}

uint32_t Buffer::GetID() const
{
    return _impl->id;
}

BufferUsage Buffer::GetUsage() const
{
    return _impl->usage;
}

size_t Buffer::GetSize() const
{
    return _impl->size;
}

void Buffer::Upload(const void* data, const size_t size, const size_t offset) const
{
    if (_impl->usage != BufferUsage::Dynamic)
    {
        spdlog::critical("Buffer ID = {} was created with BufferUsage::Static but Upload() was called - upload rejected!", _impl->id);
        return;
    }

    if (offset > _impl->size || size > (_impl->size - offset))
    {
        spdlog::critical("Buffer ID = {} size = {} bytes - attempting to upload {} bytes from {:p} at offset {} - upload rejected!", _impl->id, _impl->size, size, static_cast<const void*>(data), offset);
        return;
    }

    glBindBuffer(GL_COPY_WRITE_BUFFER, _impl->id);
    glBufferSubData(GL_COPY_WRITE_BUFFER, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(size), data);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

void Buffer::SetAsShaderResource(const uint32_t resource_type, const uint32_t binding, const size_t size, const size_t offset) const
{
    if (offset > _impl->size || size > (_impl->size - offset))
    {
        spdlog::critical("Buffer ID = {} size = {} - invalid bind range size = {} offset = {} - bind rejected!", _impl->id, _impl->size, size, offset);
        return;
    }

    glBindBufferRange(resource_type, binding, _impl->id, offset, size);
}