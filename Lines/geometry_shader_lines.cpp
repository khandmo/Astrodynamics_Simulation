#include "geometry_shader_lines.h"


#define MAX_VERTS 3 * 12 * 1024 * 1024



geom_shader_lines_device_t*
geom_shdr_lines_init_device( void )
{
  geom_shader_lines_device_t* device = (geom_shader_lines_device_t * )malloc( sizeof(geom_shader_lines_device_t ) );

  std::string vertexCode = get_file_contents("geoLine2.vert");
  std::string fragmentCode = get_file_contents("geoLine.frag");
  //std::string geometryCode = get_file_contents("geoLine.geo");

  // string to char array
  const char* vs_src = vertexCode.c_str();
  const char* fs_src = fragmentCode.c_str();

  GLuint vertex_shader = glCreateShader( GL_VERTEX_SHADER );
  GLuint fragment_shader = glCreateShader( GL_FRAGMENT_SHADER );

  glShaderSource( vertex_shader, 1, &vs_src, 0 );
  glCompileShader( vertex_shader );
  gl_utils_assert_shader_compiled( vertex_shader, "VERTEX_SHADER" );

  glShaderSource( fragment_shader, 1, &fs_src, 0 );
  glCompileShader( fragment_shader );
  gl_utils_assert_shader_compiled( fragment_shader, "FRAGMENT_SHADER" );

  device->program_id = glCreateProgram();
  glAttachShader( device->program_id, vertex_shader );
  glAttachShader( device->program_id, fragment_shader );
  glLinkProgram( device->program_id );
  gl_utils_assert_program_linked( device->program_id );
  
  glDetachShader( device->program_id, vertex_shader );
  glDetachShader( device->program_id, fragment_shader );
  glDeleteShader( vertex_shader );
  glDeleteShader( fragment_shader );

  device->attribs.pos_width1 = glGetAttribLocation( device->program_id, "pos_width1" ); // linked to index 0
  device->attribs.col1 = glGetAttribLocation( device->program_id, "col1" ); // linked to index 1
  device->attribs.pos_width2 = glGetAttribLocation(device->program_id, "pos_width2"); // linked to index 0
  device->attribs.col2 = glGetAttribLocation(device->program_id, "col2"); // linked to index 1

  device->uniforms.mvp = glGetUniformLocation( device->program_id, "u_mvp" ); // linked to index 1
  device->uniforms.viewport_size = glGetUniformLocation( device->program_id, "u_viewport_size" ); // linked to index 2
  device->uniforms.aa_radius = glGetUniformLocation(device->program_id, "u_aa_radius"); // linked to index 0


  // Generate VAO & VBO
  glGenVertexArrays(1, &device->vao);
  glGenBuffers(1, &device->vbo);
  // Bind them
  glBindVertexArray(device->vao);
  glBindBuffer(GL_ARRAY_BUFFER, device->vbo);
  // Uses buffer bound to target (GL_ARRAY_BUFFER) as data store
  glBufferData(GL_ARRAY_BUFFER, MAX_VERTS * sizeof(vertex_t), NULL, GL_DYNAMIC_DRAW);
  // How to update buffer dynamically? Call the above?
  // How to assign both attrib loc to buffer?

  glEnableVertexAttribArray(0); // binding_idx
  // 4 is for the amount of GL_FLOAT's associated with attrib
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 2*sizeof(glm::vec4), 0); // the (void*) turns decimal to hex
  glVertexAttribDivisor(0, 1);

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 2*sizeof(glm::vec4), (void*)sizeof(glm::vec4));
  glVertexAttribDivisor(1, 1);

  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec4), (void*)(2 * sizeof(glm::vec4)));
  glVertexAttribDivisor(2, 1);

  glEnableVertexAttribArray(2);
  glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec4), (void*)(3 * sizeof(glm::vec4)));
  glVertexAttribDivisor(3, 1);

  glBindVertexArray(0); // Unbind VAO


  return device;
}

void
geom_shdr_lines_term_device( void** device_in )
{
  geom_shader_lines_device_t* device = (geom_shader_lines_device_t *) *device_in;
  glDeleteProgram( device->program_id );
  glDeleteBuffers( 1, &device->vbo );
  glDeleteVertexArrays( 1, &device->vao );
  free( device );
  *device_in = NULL;
}

uint32_t
geom_shdr_lines_update(geom_shader_lines_device_t* device_in, const void* data, int32_t n_elems, int32_t elem_size,
                        uniform_data_t* uniform_data)
{
  geom_shader_lines_device_t* device = (geom_shader_lines_device_t *) device_in;
  
  device->uniform_data = uniform_data;


  glBindBuffer(GL_ARRAY_BUFFER, device->vbo);
  glBufferData(GL_ARRAY_BUFFER, n_elems * elem_size, NULL, GL_DYNAMIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, n_elems * elem_size, data);

  return n_elems;
}

void
geom_shdr_lines_render( const geom_shader_lines_device_t* device_in, const int32_t count )
{
  const geom_shader_lines_device_t* device = (geom_shader_lines_device_t * ) device_in;
  
  glUseProgram( device->program_id);

  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);


  glUniformMatrix4fv( device->uniforms.mvp, 1, GL_FALSE, device->uniform_data->mvp );
  glUniform2fv( device->uniforms.viewport_size, 1, device->uniform_data->viewport );
  glUniform2fv( device->uniforms.aa_radius, 1, device->uniform_data->aa_radius );

  glBindVertexArray( device->vao );

  glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, count - 1);

  glDisable(GL_BLEND);
  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glBindVertexArray( 0 );
  glUseProgram( 0 );
}


