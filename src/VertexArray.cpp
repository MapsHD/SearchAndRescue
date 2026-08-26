#include <cave-traversal-tool/OpenGL/VertexArray.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

uint32_t opengl_vertex_array_create_vertex_array(uint32_t vertex_buffer_id, bool has_vertex_buffer, uint32_t index_buffer_id, bool has_index_buffer, const std::vector<VertexBufferAttributeLayout>& layout)
{

    uint32_t vao_id = 0;
    glCreateVertexArrays(1, &vao_id);

    if (has_index_buffer)
    {
        glVertexArrayElementBuffer(vao_id, index_buffer_id);
    }

    if (!has_vertex_buffer)
    {
        spdlog::error("Attempt to create VAO without VBO!");
        std::abort();
    }

    for (const auto& attribute_layout : layout)
    {
        const auto location   = attribute_layout.location;
        const auto components = attribute_layout.components;
        const auto type       = attribute_layout.type;
        const auto normalize  = attribute_layout.normalize;
        const auto stride     = attribute_layout.stride;
        const auto offset     = attribute_layout.offset;

        glVertexArrayVertexBuffer(vao_id, location, vertex_buffer_id, offset, stride);
        glEnableVertexArrayAttrib(vao_id, location);
        glVertexArrayAttribFormat(vao_id, location, components, type, normalize, 0);
        glVertexArrayAttribBinding(vao_id, location, location);
    }

    return vao_id;
}