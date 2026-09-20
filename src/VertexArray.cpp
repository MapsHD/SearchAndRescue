#include <cave-traversal-tool/OpenGL/VertexArray.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct VertexArray::VertexArrayIMPL
{
    uint32_t id = {};

    Buffer* vbo           = {};
    bool    vbo_ownership = {};

    Buffer* ibo           = {};
    bool    ibo_ownership = {};
};

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

void VertexArray::DrawElements(const uint32_t mode, const uint32_t index_count, const uint32_t instance_count, const uint32_t base_instance)
{
    glDrawElementsInstancedBaseVertexBaseInstance(mode, index_count, GL_UNSIGNED_INT, nullptr, instance_count, 0, base_instance);
}
