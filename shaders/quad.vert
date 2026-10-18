#version 450
layout(location = 0) out vec2 uv;
void main()
{
    // fullscreen triangle-strip quad: pos = f(idx)
    float x = (gl_VertexIndex == 1 || gl_VertexIndex == 3) ? 1.0 : -1.0;
    float y = (gl_VertexIndex >= 2) ? 1.0 : -1.0;
    // flip Y for texture space
    uv = vec2((x + 1.0) * 0.5, (1.0 - y) * 0.5);
    gl_Position = vec4(x, y, 0.0, 1.0);
}
