#include <cave-traversal-tool/OpenGL/Buffer.h>

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

    glCreateBuffers(1, &_impl->id);
    glNamedBufferStorage(_impl->id, _impl->size, data, _impl->flags);
}

Buffer::~Buffer()
{
    glInvalidateBufferData(_impl->id);
    glDeleteBuffers(1, &_impl->id);

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
    if ((_impl->flags & GL_DYNAMIC_STORAGE_BIT) == 0)
    {
        spdlog::critical("Buffer ID = {} was created without GL_DYNAMIC_STORAGE_BIT but Upload() was called - upload rejected!", _impl->id);
        return;
    }

    if (offset > _impl->size || size > (_impl->size - offset))
    {
        spdlog::critical("Buffer ID = {} size = {} bytes - attempting to upload {} bytes from {:p} at offset {} - upload rejected!", _impl->id, _impl->size, size, static_cast<const void*>(data), offset);
        return;
    }

    glNamedBufferSubData(_impl->id, offset, size, data);
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