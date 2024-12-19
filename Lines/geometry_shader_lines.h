#pragma once

#ifndef GEOMETRY_SHADER_LINES_H
#define GEOMETRY_SHADER_LINES_H

#include "../Shaders.h"
#include <glad/glad.h>
#include "../gl_utils.h"
#include <GLFW/glfw3.h>
#include <stdbool.h>
#include <cassert>
#include <glm/glm.hpp>


typedef struct vertex
{
    union
    {
        struct { glm::vec3 pos; float width; };
        glm::vec4 pos_width;
    };
    glm::vec4 col;
} vertex_t;

typedef struct uniform_data
{
    float* mvp;
    float* viewport;
    float* aa_radius;
} uniform_data_t;

typedef struct geom_shader_lines_device
{
    GLuint program_id;
    GLuint vao;
    GLuint vbo;

    struct geom_shader_lines_uniform_locations
    {
        GLuint mvp;
        GLuint viewport_size;
        GLuint aa_radius;
    } uniforms;

    struct geom_shader_lines_attrib_locations
    {
        GLuint pos_width;
        GLuint col;
    } attribs;

    uniform_data_t* uniform_data;
} geom_shader_lines_device_t
;

geom_shader_lines_device_t geom_shdr_lines_init_device( void );
uint32_t geom_shdr_lines_update(geom_shader_lines_device_t* device, const void* data, int32_t n_elems, int32_t elem_size,
                                 uniform_data_t* uniform_data, int32_t bIdx, int32_t rIdx );
void geom_shdr_lines_render( const geom_shader_lines_device_t* device, const int32_t count );
void geom_shdr_lines_term_device( void** device );

#endif /* GEOMETRY_SHADER_LINES_H */

