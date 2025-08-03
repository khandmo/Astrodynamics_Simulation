#version 330 core
layout(location = 0) in vec4 pos_width1; // xyz = position, w = width
layout(location = 1) in vec4 color1;
layout(location = 2) in vec4 pos_width2;
layout(location = 3) in vec4 color2;

uniform mat4 u_mvp;
uniform vec2 u_viewport_size;
uniform vec2 u_aa_radius;

out vec4 g_col;
noperspective out float g_u;
noperspective out float g_v;
noperspective out float g_line_width;
noperspective out float g_line_length;

void main()
{
    int corner = gl_VertexID % 4;
    int lineID = gl_VertexID / 4;

    int baseIndex = lineID * 2;

    // These would be fetched from buffer in a full setup; simulate behavior here
    vec4 p0 = pos_width1; // placeholder for simulation
    vec4 p1 = pos_width2; // placeholder for simulation
    vec3 a = p0.xyz;
    float w0 = p0.w;
    vec3 b = p1.xyz;
    float w1 = p1.w;

    float width0 = max(w0, 1.0) + u_aa_radius.x;
    float width1 = max(w1, 1.0) + u_aa_radius.x;

    vec4 clip_a = u_mvp * vec4(a, 1.0);
    vec4 clip_b = u_mvp * vec4(b, 1.0);

    vec2 ndc_a = clip_a.xy / clip_a.w;
    vec2 ndc_b = clip_b.xy / clip_b.w;

    vec2 screen_size = u_viewport_size;
    float aspect = screen_size.y / screen_size.x;

    vec2 dir = normalize((ndc_b - ndc_a) * vec2(1.0, aspect));
    vec2 normal = vec2(-dir.y, dir.x);

    float ext = u_aa_radius.y / screen_size.x;
    float line_length = length((ndc_b - ndc_a) * screen_size) + 2.0 * u_aa_radius.y;

    vec2 offset;
    vec2 ndc_pos;
    float z;
    float w;
    float gwidth;
    float gu;
    float gv;

    if (corner == 0) {
        offset = normal * (width0 / screen_size);
        ndc_pos = ndc_a - offset - dir * ext;
        z = clip_a.z;
        w = clip_a.w;
        gwidth = width0;
        gu = -width0;
        gv = line_length * 0.5;
    } else if (corner == 1) {
        offset = normal * (width0 / screen_size);
        ndc_pos = ndc_a + offset - dir * ext;
        z = clip_a.z;
        w = clip_a.w;
        gwidth = width0;
        gu = width0;
        gv = line_length * 0.5;
    } else if (corner == 2) {
        offset = normal * (width1 / screen_size);
        ndc_pos = ndc_b - offset + dir * ext;
        z = clip_b.z;
        w = clip_b.w;
        gwidth = width1;
        gu = -width1;
        gv = -line_length * 0.5;
    } else {
        offset = normal * (width1 / screen_size);
        ndc_pos = ndc_b + offset + dir * ext;
        z = clip_b.z;
        w = clip_b.w;
        gwidth = width1;
        gu = width1;
        gv = -line_length * 0.5;
    }

    gl_Position = vec4(ndc_pos * w, z, w);
    g_col = color1;
    g_u = gu;
    g_v = gv;
    g_line_width = gwidth;
    g_line_length = line_length * 0.5;
}
