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

// ---------------------------------------------------------------------------
// Internal helper: attach one buffer binding to the VAO
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

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
VertexArray::VertexArray(const std::vector<VertexBufferBinding>& bindings, Buffer* index_buffer, const bool index_buffer_ownership)
{
    _impl = new VertexArrayIMPL;

    _impl->id            = UINT32_MAX;
    _impl->ibo           = index_buffer;
    _impl->ibo_ownership = index_buffer_ownership;

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
