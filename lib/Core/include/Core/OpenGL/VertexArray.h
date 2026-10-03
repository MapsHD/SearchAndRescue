#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Core/OpenGL/Buffer.h>
#include <Core/Structures.h>

struct VertexBufferAttributeLayout
{
    uint32_t location;
    int32_t  components;
    uint32_t type;
    int32_t  normalize;
    int32_t  stride;
    int32_t  offset;
};

template <typename T>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout();

template <>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<ColorPoint>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(ColorPoint), offsetof(ColorPoint, position)},
            {1, 3, /* GL_UNSIGNED_BYTE */ 0x1401, /* GL_TRUE */ 1, sizeof(ColorPoint), offsetof(ColorPoint, color)}};
}

template <>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<Point>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(Point), offsetof(Point, position)}};
}

template <>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<ColoredVertex>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(ColoredVertex), offsetof(ColoredVertex, position)},
            {1, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(ColoredVertex), offsetof(ColoredVertex, color)}};
}

template <>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<PointIntensity>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(PointIntensity), offsetof(PointIntensity, position)},
            {1, 1, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(PointIntensity), offsetof(PointIntensity, intensity)}};
}

// Associates a single VBO with a set of vertex attribute layouts.
// binding_index is the VAO vertex-buffer binding slot to use.
// When UINT32_MAX (the default) the first layout entry's location is used as
// the binding slot, which preserves the legacy one-buffer-per-attribute behaviour.
struct VertexBufferBinding
{
    Buffer*                                  buffer           = nullptr;
    bool                                     buffer_ownership = false;
    std::vector<VertexBufferAttributeLayout> layout           = {};
    uint32_t                                 binding_index    = UINT32_MAX;
};

class VertexArray
{
private:
    struct VertexArrayIMPL;

private:
    VertexArrayIMPL* _impl = nullptr;

public:
    // Legacy single-VBO constructor (index buffer optional).
    explicit VertexArray(Buffer* vertex_buffer, const bool vertex_buffer_ownership, Buffer* index_buffer, const bool index_buffer_ownership, const std::vector<VertexBufferAttributeLayout>& layout);

    // Multi-VBO constructor: each VertexBufferBinding is attached as a separate
    // buffer slot.  The optional index_buffer is not owned by any binding entry.
    explicit VertexArray(const std::vector<VertexBufferBinding>& bindings, Buffer* index_buffer, const bool index_buffer_ownership);

    ~VertexArray();

    [[nodiscard]] uint32_t GetID() const;

    void Bind();
    void Unbind();

    void DrawArray(const uint32_t mode, const uint32_t vertex_count);
    void DrawArray(const uint32_t mode, const uint32_t first, const uint32_t vertex_count);
    void DrawElements(const uint32_t mode, const uint32_t index_count);
};
