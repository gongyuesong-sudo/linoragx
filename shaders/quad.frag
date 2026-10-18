#version 450
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outc;
layout(set = 0, binding = 0) uniform sampler2D tex;
layout(push_constant) uniform PC { int scanlines; } pc;
void main()
{
    outc = texture(tex, uv);
    if (pc.scanlines == 1) {
        // darken every other source line (224 lines scaled)
        float sy = uv.y * 224.0;
        if (fract(sy) > 0.5) outc.rgb *= 0.65;
    }
}
