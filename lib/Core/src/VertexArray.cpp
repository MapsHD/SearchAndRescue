#include <Core/OpenGL/VertexArray.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct VertexArray::VertexArrayIMPL
{
    uint32_t id = {};

    // Legacy single-VBO path
    Buffer* vbo           = {};
    bool    vbo_ownership = {};

    // Multi-VBO path : owned buffer pointers
    std::vector<Buffer*> owned_vbos = {};

    Buffer* ibo           = {};
    bool    ibo_ownership = {};
};

// ---------------------------------------------------------------------------
// Internal helper: attach one buffer binding slot to the VAO
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

// ---------------------------------------------------------------------------
// Legacy single-VBO constructor
// ---------------------------------------------------------------------------
VertexArray::VertexArray(Buffer* vertex_buffer, const bool vertex_buffer_ownership, Buffer* index_buffer, const bool index_buffer_ownership, const std::vector<VertexBufferAttributeLayout>& layout)
{
    _impl = new VertexArrayIMPL;

    _impl->id            = UINT32_MAX;
    _impl->vbo           = vertex_buffer;
    _impl->vbo_ownership = vertex_buffer_ownership;
    _impl->ibo           = index_buffer;
    _impl->ibo_ownership = index_buffer_ownership;

    glCreateVertexArrays(1, &_impl->id);

    if (_impl->ibo)
    {
        glVertexArrayElementBuffer(_impl->id, _impl->ibo->GetID());
    }

    if (!_impl->vbo)
    {
        return;
    }

    for (const VertexBufferAttributeLayout& attribute_layout : layout)
    {
        const auto& [location, components, type, normalize, stride, offset] = attribute_layout;

        glVertexArrayVertexBuffer(_impl->id, location, _impl->vbo->GetID(), offset, stride);
        glEnableVertexArrayAttrib(_impl->id, location);
        glVertexArrayAttribFormat(_impl->id, location, components, type, static_cast<uint8_t>(normalize), 0);
        glVertexArrayAttribBinding(_impl->id, location, location);
    }
}

// ---------------------------------------------------------------------------
// Multi-VBO constructor
// ---------------------------------------------------------------------------
VertexArray::VertexArray(const std::vector<VertexBufferBinding>& bindings, Buffer* index_buffer, const bool index_buffer_ownership)
{
    _impl = new VertexArrayIMPL;

    _impl->id            = UINT32_MAX;
    _impl->vbo           = nullptr;
    _impl->vbo_ownership = false;
    _impl->ibo           = index_buffer;
    _impl->ibo_ownership = index_buffer_ownership;

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

        // Determine the binding slot: explicit value or fall back to the first
        // attribute's location (legacy behaviour).
        const uint32_t slot = (binding.binding_index != UINT32_MAX)
                                  ? binding.binding_index
                                  : binding.layout.front().location;

        attach_buffer_binding(_impl->id, binding.buffer, binding.layout, slot);

        if (binding.buffer_ownership)
        {
            _impl->owned_vbos.push_back(binding.buffer);
        }
    }
}

VertexArray::~VertexArray()
{
    if (_impl->ibo_ownership && _impl->ibo)
    {
        delete _impl->ibo;
    }

    if (_impl->vbo_ownership && _impl->vbo)
    {
        delete _impl->vbo;
    }

    for (Buffer* buf : _impl->owned_vbos)
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
