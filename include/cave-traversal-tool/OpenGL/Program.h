#pragma once

#include <cstdint>
#include <vector>

uint32_t opengl_program_compile_shader(const std::vector<char>& source, uint32_t shader_type);
uint32_t opengl_program_create_program(uint32_t vertex_shader_id, uint32_t fragment_shader_id);