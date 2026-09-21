#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <cave-traversal-tool/OpenGL/Buffer.h>
#include <cave-traversal-tool/Structures.h>

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
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<PointIntensity>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(PointIntensity), offsetof(PointIntensity, position)},
            {1, 1, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(PointIntensity), offsetof(PointIntensity, intensity)}};
}

template <>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<NormalPoint>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(NormalPoint), offsetof(NormalPoint, position)},
            {1, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(NormalPoint), offsetof(NormalPoint, normal)}};
}

class VertexArray
{
private:
    struct VertexArrayIMPL;

private:
    VertexArrayIMPL* _impl = nullptr;

public:
    explicit VertexArray(Buffer* vertex_buffer, const bool vertex_buffer_ownership, Buffer* index_buffer, const bool index_buffer_ownership, const std::vector<VertexBufferAttributeLayout>& layout);
    ~VertexArray();

    [[nodiscard]] uint32_t GetID() const;

    void Bind();
    void Unbind();

    void DrawArray(const uint32_t mode, const uint32_t vertex_count);
    void DrawElements(const uint32_t mode, const uint32_t index_count, const uint32_t instance_count, const uint32_t base_instance);
};
