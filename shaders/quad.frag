#version 450
/* DLSS-style visual enhancement.
 *
 * Algorithms ported from dlss5-visual-enhancer 21.0 (src/processing/shaders):
 *   - Lanczos4 resampling filter   (filters_0.glsl.in)
 *   - AMD FidelityFX CAS sharpen   (sharpening_1.glsl.in, sRGB transfer, sharpness 50)
 *   - directional luma USM         (sharpening_2/3.glsl.in, slider 0 params)
 * Single-pass equivalent of their scale->sharpen stage chain: sharpen taps
 * step by 1/ratio source texels, matching sharpening applied on the upscaled frame.
 *
 * pc.dlss: 0 = plain nearest (legacy)
 *          1 = Lanczos4 upscale + CAS sharpen
 *          2 = Lanczos4 upscale + directional USM sharpen
 */
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outc;
layout(set = 0, binding = 0) uniform sampler2D tex;
layout(push_constant) uniform PC {
    int scanlines;
    int dlss;
    float sharp;
    float _pad;
    vec2 dstsz;
} pc;

vec3 s2l(vec3 c) {
    return mix(pow((c + 0.055) / 1.055, vec3(2.4)), c / 12.92, lessThanEqual(c, vec3(0.04045)));
}
vec3 l2s(vec3 c) {
    return mix(1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, c * 12.92, lessThanEqual(c, vec3(0.0031308)));
}
vec4 px(ivec2 q) {
    ivec2 s = textureSize(tex, 0);
    return texelFetch(tex, clamp(q, ivec2(0), s - ivec2(1)), 0);
}
float lanczos(float x) {
    x = abs(x);
    if (x >= 4.0) return 0.0;
    if (x >= 1.0) { float h = x * 0.5; return 4.0 * sin(3.14159265 * x) * sin(3.14159265 * h) / (3.14159265 * 3.14159265 * x * h); }
    return x == 0.0 ? 1.0 : sin(3.14159265 * x) / (3.14159265 * x);
}

/* Lanczos4 resampling, per filters_0.glsl.in */
vec3 lanczos_up(vec2 srcsz, vec2 dstsz)
{
    vec2 ratio = srcsz / dstsz;
    vec2 p = uv * srcsz - 0.5;
    vec2 support = max(ratio, vec2(1.0));
    vec3 sum = vec3(0.0);
    float total = 0.0;
    int y0 = int(ceil(p.y - 4.0 * support.y)), y1 = int(floor(p.y + 4.0 * support.y));
    int x0 = int(ceil(p.x - 4.0 * support.x)), x1 = int(floor(p.x + 4.0 * support.x));
    for (int y = y0; y <= y1; y++) {
        float wy = lanczos((float(y) - p.y) / support.y);
        for (int x = x0; x <= x1; x++) {
            float w = wy * lanczos((float(x) - p.x) / support.x);
            sum += px(ivec2(x, y)).rgb * w;
            total += w;
        }
    }
    return clamp(sum / max(total, 1e-20), 0.0, 1.0);
}

/* CAS, per sharpening_1.glsl.in: contrast-limited 5-tap cross, linear light.
 * Taps step by `st` source texels = one destination pixel. sharpness 50. */
vec3 cas_sharpen(ivec2 c, float st)
{
    vec3 E = s2l(px(c).rgb);
    vec3 B = s2l(px(c + ivec2(0, -int(st))).rgb);
    vec3 D = s2l(px(c + ivec2(-int(st), 0)).rgb);
    vec3 F = s2l(px(c + ivec2(int(st), 0)).rgb);
    vec3 H = s2l(px(c + ivec2(0, int(st))).rgb);
    float mn = min(min(min(D.g, E.g), F.g), min(B.g, H.g));
    float mx = max(max(max(D.g, E.g), F.g), max(B.g, H.g));
    float w = sqrt(clamp(min(mn, 1.0 - mx) / max(mx, 1e-20), 0.0, 1.0)) * (-1.0 / 6.5);
    vec3 result = clamp((E + w * (B + D + F + H)) / (1.0 + 4.0 * w), 0.0, 1.0);
    return l2s(mix(E, result, 0.5));
}

float usm1(float a, float b, float c, float d, float e, float strength, float limit)
{
    float ac = max(max(a, b), c) - min(min(a, b), c);
    float bc = max(max(c, d), e) - min(min(c, d), e);
    float ratio = max(ac, bc) / (min(ac, bc) + 1.0 / 255.0);
    float lti = 1.0 - clamp((ratio - 2.0) / 8.0, 0.0, 1.0);
    return clamp((-0.6001 * b + 1.2002 * c - 0.6001 * d) * strength, -limit, limit) * lti;
}

/* directional luma USM, per sharpening_2/3.glsl.in (slider 0 params), taps step by st */
vec3 dir_usm(ivec2 c, float st)
{
    vec4 center = px(c);
    float p[25];
    for (int y = 0; y < 5; y++)
        for (int x = 0; x < 5; x++)
            p[y * 5 + x] = dot(px(c + (ivec2(x, y) - 2) * int(st)).rgb, vec3(0.2126, 0.7152, 0.0722));
    float g0   = abs(p[6] + p[7] + p[8] - p[16] - p[17] - p[18]);
    float g45  = abs(p[11] + p[6] + p[7] - p[17] - p[18] - p[13]);
    float g90  = abs(p[6] + p[11] + p[16] - p[8] - p[13] - p[18]);
    float g135 = abs(p[11] + p[16] + p[17] - p[7] - p[8] - p[13]);
    float ax = max(g0, g90), an = min(g0, g90), dx = max(g45, g135), dn = min(g45, g135);
    bool ad = ax > an * (2.0 * 1127.0 / 1024.0) && ax > 64.0 / 1024.0 && ax > dn;
    bool dd = dx > dn * (2.0 * 1127.0 / 1024.0) && dx > 64.0 / 1024.0 && dx > an;
    float af = ad && dd ? ax / max(ax + dx, 1e-20) : 1.0;
    float df = ad && dd ? dx / max(ax + dx, 1e-20) : 1.0;
    float scale = 1.0 - clamp((p[12] - 0.45) / (0.9 - 0.45), 0.0, 1.0);
    float strength = scale * 1.2 + 0.4;
    float limit = (scale * 0.36 + 0.14) * p[12];
    float sum = 0.0;
    if (ad) sum += af * (g0 >= g90 ? usm1(p[2], p[7], p[12], p[17], p[22], strength, limit)
                                   : usm1(p[10], p[11], p[12], p[13], p[14], strength, limit));
    if (dd) sum += df * (g45 >= g135 ? usm1(p[6], 0.5 * (p[11] + p[7]), p[12], 0.5 * (p[17] + p[13]), p[18], strength, limit)
                                     : usm1(p[16], 0.5 * (p[17] + p[11]), p[12], 0.5 * (p[13] + p[7]), p[8], strength, limit));
    return clamp(center.rgb + sum, 0.0, 1.0);
}

void main()
{
    vec3 col;
    if (pc.dlss == 0) {
        col = texture(tex, uv).rgb;
    } else {
        vec2 srcsz = vec2(textureSize(tex, 0));
        vec2 dstsz = pc.dstsz.x > 0.0 ? pc.dstsz : srcsz * 2.0;
        ivec2 c = ivec2(floor(uv * srcsz));
        float st = max(1.0, floor(srcsz.x / dstsz.x + 0.5));   /* sharpen tap step */
        col = lanczos_up(srcsz, dstsz);
        /* sharpen in source space, blend over the resampled base so the
           upscale detail is kept: base + (sharpened_center - center) */
        vec3 base = px(c).rgb;
        vec3 sharp = pc.dlss >= 2 ? dir_usm(c, st) : cas_sharpen(c, st);
        col = clamp(col + (sharp - base), 0.0, 1.0);
    }
    outc = vec4(col, 1.0);
    if (pc.scanlines == 1) {
        // darken every other source line (224 lines scaled)
        float sy = uv.y * 224.0;
        if (fract(sy) > 0.5) outc.rgb *= 0.65;
    }
}
