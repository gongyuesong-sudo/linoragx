#version 450
/* Pixel-art enhancement: generalized Scale2x (Hyllian EPX corner rules
 * applied at arbitrary scale). Flat texels stay solid color; texels that
 * form staircase jaggies get their corners filled per the Scale2x pattern
 * rules, then sub-texel bilinear makes the staircase read as a smooth
 * curve. Only existing source colors are mixed: no noise, no ringing,
 * no new frequencies, no invented detail.
 *
 * pc.enhance: 0 = plain nearest (legacy), 1 = Scale2x (default).
 * Letterbox is handled C-side (viewport fits 320:224, bars = clear color),
 * so uv here always spans exactly the game image. */
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outc;
layout(set = 0, binding = 0) uniform sampler2D tex;
layout(push_constant) uniform PC {
    int scanlines;
    int enhance;
} pc;

vec4 px(ivec2 q) {
    ivec2 s = textureSize(tex, 0);
    return texelFetch(tex, clamp(q, ivec2(0), s - ivec2(1)), 0);
}

void main()
{
    vec3 col;
    if (pc.enhance == 0) {
        col = texture(tex, uv).rgb;
    } else {
        vec2 srcsz = vec2(textureSize(tex, 0));
        vec2 p = uv * srcsz - 0.5;
        vec2 fl = floor(p);
        ivec2 c = ivec2(fl);
        vec2 f = p - fl;
        vec3 A = px(c + ivec2(-1,-1)).rgb, B = px(c + ivec2(0,-1)).rgb, C = px(c + ivec2(1,-1)).rgb;
        vec3 D = px(c + ivec2(-1, 0)).rgb, E = px(c).rgb,              F = px(c + ivec2(1, 0)).rgb;
        vec3 G = px(c + ivec2(-1, 1)).rgb, H = px(c + ivec2(0, 1)).rgb, I = px(c + ivec2(1, 1)).rgb;
        /* Scale2x corner rules */
        vec3 E0 = (D == A && A != C && D != G) ? A : E;
        vec3 E1 = (B == C && A != C && B != F) ? C : E;
        vec3 E2 = (D == G && D != I && G != H) ? G : E;
        vec3 E3 = (H == F && D != I && H != F) ? F : E;
        col = mix(mix(E0, E1, f.x), mix(E2, E3, f.x), f.y);
    }
    outc = vec4(col, 1.0);
    if (pc.scanlines == 1) {
        // darken every other source line (224 lines scaled)
        float sy = uv.y * 224.0;
        if (fract(sy) > 0.5) outc.rgb *= 0.65;
    }
}
