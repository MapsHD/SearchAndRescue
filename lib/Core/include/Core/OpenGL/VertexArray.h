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

// Orientation axis columns fed as vertex attributes to the TrajectoryOrientations shader.
// Locations 1/2/3 correspond to in_AxisX / in_AxisY / in_AxisZ.
template <>
inline std::vector<VertexBufferAttributeLayout> opengl_vertex_array_get_vertex_layout<OrientationAxesVertex>()
{
    return {{1, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(OrientationAxesVertex), offsetof(OrientationAxesVertex, axis_x)},
            {2, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(OrientationAxesVertex), offsetof(OrientationAxesVertex, axis_y)},
            {3, 3, /* GL_FLOAT */ 0x1406, /* GL_FALSE */ 0, sizeof(OrientationAxesVertex), offsetof(OrientationAxesVertex, axis_z)}};
}

// Associates a VBO with a set of attribute layouts and an explicit binding slot.
// All attributes in layout share the same buffer binding slot; their individual
// byte offsets are expressed as relativeoffset in glVertexArrayAttribFormat.
struct VertexBufferBinding
{
    Buffer*                                  buffer           = nullptr;
    bool                                     buffer_ownership = false;
    std::vector<VertexBufferAttributeLayout> layout           = {};
    uint32_t                                 binding_index    = 0;
};

class VertexArray
{
private:
    struct VertexArrayIMPL;

private:
    VertexArrayIMPL* _impl = nullptr;

public:
    // Constructs a VAO from one or more buffer bindings.
    // Each VertexBufferBinding attaches a VBO to a specific binding slot with its attribute layout.
    // The optional index_buffer is used for indexed draw calls and is not owned by any binding.
    explicit VertexArray(const std::vector<VertexBufferBinding>& bindings, Buffer* index_buffer, const bool index_buffer_ownership);

    ~VertexArray();

    [[nodiscard]] uint32_t GetID() const;

    void Bind();
    void Unbind();

    void DrawArray(const uint32_t mode, const uint32_t vertex_count);
    void DrawArray(const uint32_t mode, const uint32_t first, const uint32_t vertex_count);
    void DrawElements(const uint32_t mode, const uint32_t index_count);
};
