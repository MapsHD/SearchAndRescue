#include <Core/OpenGL/Buffer.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct Buffer::BufferIMPL
{
    uint32_t id    = {};
    uint32_t flags = {};
    size_t   size  = {};
};

Buffer::Buffer(const uint32_t flags, const size_t size, const void* data)
{
    _impl = new BufferIMPL;

    _impl->id    = UINT32_MAX;
    _impl->flags = flags;
    _impl->size  = size;

#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
    GLenum usage = GL_STATIC_DRAW;
    if ((flags & GL_DYNAMIC_STORAGE_BIT) != 0 || flags == GL_DYNAMIC_DRAW)
    {
        usage = GL_DYNAMIC_DRAW;
    }
    else if (flags == GL_STREAM_DRAW)
    {
        usage = GL_STREAM_DRAW;
    }
    else if (flags == GL_STATIC_DRAW)
    {
        usage = GL_STATIC_DRAW;
    }
    else if (flags != 0 && flags != GL_NONE)
    {
        usage = flags;
    }

    glGenBuffers(1, &_impl->id);
    glBindBuffer(GL_COPY_WRITE_BUFFER, _impl->id);
    glBufferData(GL_COPY_WRITE_BUFFER, static_cast<GLsizeiptr>(_impl->size), data, usage);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
#else
    glCreateBuffers(1, &_impl->id);
    glNamedBufferStorage(_impl->id, _impl->size, data, _impl->flags);
#endif
}

Buffer::~Buffer()
{
#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
    glDeleteBuffers(1, &_impl->id);
#else
    glInvalidateBufferData(_impl->id);
    glDeleteBuffers(1, &_impl->id);
#endif

    delete _impl;
}

uint32_t Buffer::GetID() const
{
    return _impl->id;
}

uint32_t Buffer::GetFlags() const
{
    return _impl->flags;
}

size_t Buffer::GetSize() const
{
    return _impl->size;
}

void Buffer::Upload(const void* data, const size_t size, const size_t offset) const
{
#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
    if ((_impl->flags & GL_DYNAMIC_STORAGE_BIT) == 0 && _impl->flags != GL_DYNAMIC_DRAW && _impl->flags != GL_STREAM_DRAW)
    {
        spdlog::critical("Buffer ID = {} was created without dynamic usage but Upload() was called - upload rejected!", _impl->id);
        return;
    }
#else
    if ((_impl->flags & GL_DYNAMIC_STORAGE_BIT) == 0)
    {
        spdlog::critical("Buffer ID = {} was created without GL_DYNAMIC_STORAGE_BIT but Upload() was called - upload rejected!", _impl->id);
        return;
    }
#endif

    if (offset > _impl->size || size > (_impl->size - offset))
    {
        spdlog::critical("Buffer ID = {} size = {} bytes - attempting to upload {} bytes from {:p} at offset {} - upload rejected!", _impl->id, _impl->size, size, static_cast<const void*>(data), offset);
        return;
    }

#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
    glBindBuffer(GL_COPY_WRITE_BUFFER, _impl->id);
    glBufferSubData(GL_COPY_WRITE_BUFFER, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(size), data);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
#else
    glNamedBufferSubData(_impl->id, offset, size, data);
#endif
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