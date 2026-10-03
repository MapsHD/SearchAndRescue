#include <Core/OpenGL/VertexArray.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct VertexArray::VertexArrayIMPL
{
    uint32_t id = {};

    // Buffers owned by this VAO (those whose binding had buffer_ownership=true)
    std::vector<Buffer*> owned_buffers = {};

    Buffer* ibo           = {};
    bool    ibo_ownership = {};
};

#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
// ---------------------------------------------------------------------------
// Internal helper: attach one buffer binding to the VAO (OpenGL 4.1)
// ---------------------------------------------------------------------------
static void attach_buffer_binding(Buffer* buffer, const std::vector<VertexBufferAttributeLayout>& layout)
{
    if (!buffer || layout.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, buffer->GetID());

    for (const VertexBufferAttributeLayout& attribute_layout : layout)
    {
        const auto& [location, components, type, normalize, stride, offset] = attribute_layout;

        glEnableVertexAttribArray(location);
        glVertexAttribPointer(
            location,
            components,
            type,
            static_cast<GLboolean>(normalize),
            stride,
            reinterpret_cast<const void*>(static_cast<uintptr_t>(offset)));
    }
}
#else
// ---------------------------------------------------------------------------
// Internal helper: attach one buffer binding slot to the VAO (OpenGL 4.6 DSA)
// ---------------------------------------------------------------------------
static void attach_buffer_binding(uint32_t vao_id, Buffer* buffer, const std::vector<VertexBufferAttributeLayout>& layout, uint32_t binding_slot)
{
    if (layout.empty())
    {
        return;
    }

    // Bind the buffer to the slot once with base offset=0 and the shared stride.
    // Each attribute's individual byte offset is expressed as relativeoffset below.
    glVertexArrayVertexBuffer(vao_id, binding_slot, buffer->GetID(), 0, layout.front().stride);

    for (const VertexBufferAttributeLayout& attribute_layout : layout)
    {
        const auto& [location, components, type, normalize, stride, offset] = attribute_layout;

        glEnableVertexArrayAttrib(vao_id, location);
        // relativeoffset is the per-attribute byte offset within the vertex record
        glVertexArrayAttribFormat(vao_id, location, components, type, static_cast<uint8_t>(normalize), static_cast<uint32_t>(offset));
        glVertexArrayAttribBinding(vao_id, location, binding_slot);
    }
}
#endif

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
VertexArray::VertexArray(const std::vector<VertexBufferBinding>& bindings, Buffer* index_buffer, const bool index_buffer_ownership)
{
    _impl = new VertexArrayIMPL;

    _impl->id            = UINT32_MAX;
    _impl->ibo           = index_buffer;
    _impl->ibo_ownership = index_buffer_ownership;

#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
    glGenVertexArrays(1, &_impl->id);
    glBindVertexArray(_impl->id);

    if (_impl->ibo)
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _impl->ibo->GetID());
    }

    for (const VertexBufferBinding& binding : bindings)
    {
        if (!binding.buffer || binding.layout.empty())
        {
            continue;
        }

        attach_buffer_binding(binding.buffer, binding.layout);

        if (binding.buffer_ownership)
        {
            _impl->owned_buffers.push_back(binding.buffer);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
#else
    glCreateVertexArrays(1, &_impl->id);

    if (_impl->ibo)
    {
        glVertexArrayElementBuffer(_impl->id, _impl->ibo->GetID());
    }

    for (const VertexBufferBinding& binding : bindings)
    {
        if (!binding.buffer || binding.layout.empty())
        {
            continue;
        }

        attach_buffer_binding(_impl->id, binding.buffer, binding.layout, binding.binding_index);

        if (binding.buffer_ownership)
        {
            _impl->owned_buffers.push_back(binding.buffer);
        }
    }
#endif
}

VertexArray::~VertexArray()
{
    if (_impl->ibo_ownership && _impl->ibo)
    {
        delete _impl->ibo;
    }

    for (Buffer* buf : _impl->owned_buffers)
    {
        delete buf;
    }

    glDeleteVertexArrays(1, &_impl->id);

    delete _impl;
}

uint32_t VertexArray::GetID() const
{
    return _impl->id;
}

void VertexArray::Bind()
{
    glBindVertexArray(_impl->id);
}

void VertexArray::Unbind()
{
    glBindVertexArray(0);
}

void VertexArray::DrawArray(const uint32_t mode, const uint32_t vertex_count)
{
    glDrawArrays(mode, 0, vertex_count);
}

void VertexArray::DrawArray(const uint32_t mode, const uint32_t first, const uint32_t vertex_count)
{
    glDrawArrays(mode, first, vertex_count);
}

void VertexArray::DrawElements(const uint32_t mode, const uint32_t index_count)
{
    glDrawElements(mode, index_count, GL_UNSIGNED_INT, nullptr);
}
