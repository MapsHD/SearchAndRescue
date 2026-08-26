#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

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
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<NormalPoint>()
{
    return {{0, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(NormalPoint), offsetof(NormalPoint, position)},
            {1, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(NormalPoint), offsetof(NormalPoint, normal)}};
}

uint32_t opengl_vertex_array_create_vertex_array(uint32_t vertex_buffer_id, bool has_vertex_buffer, uint32_t index_buffer_id, bool has_index_buffer, const std::vector<VertexBufferAttributeLayout>& layout);