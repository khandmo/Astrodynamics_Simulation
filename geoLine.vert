#version 330 core

    layout(location = 0) in vec4 pos_width;
    layout(location = 1) in vec4 col;

    uniform mat4 u_mvp; // had layout location = 0
      
    out vec4 v_col;
    noperspective out float v_line_width; // issue here w/ noperspective *after* in/out

    void main()
    {
    v_col = col;
    v_line_width = pos_width.w;
    gl_Position = u_mvp * vec4(pos_width.xyz, 1.0);
}