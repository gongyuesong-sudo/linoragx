#version 450
/* DLSS-style enhancement pass.
 *
 * Structure follows the OpenDLSS-NR pipeline (preprocess -> enhance -> composite,
 * docs/frame.md): an edge-aware spatial stage replaces the generative network
 * (NVIDIA's proprietary weights and FP8 tensor cores are not available here),
 * and the contrast-adaptive limiter mirrors the RCAS composite step.
 *
 * pc.dlss: 0 = plain nearest (legacy), 1 = edge-aware upscale + sharpen,
 *          2 = also diagonal-edge interpolation, stronger sharpen.
 * pc.sharp: sharpen amount (0 disables the sharpen stage).
 */
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outc;
layout(set = 0, binding = 0) uniform sampler2D tex;
layout(push_constant) uniform PC {
    int scanlines;
    int dlss;
    float sharp;
    float _pad;
} pc;

float lum(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }

void main()
{
    vec3 col;
    if (pc.dlss == 0) {
        col = texture(tex, uv).rgb;
    } else {
        vec2 srcsz = vec2(textureSize(tex, 0));
        vec2 px = uv * srcsz;
        ivec2 c0 = ivec2(floor(px)) - 1;
        vec3 c[3][3];
        float l[3][3];
        for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++) {
                ivec2 q = clamp(c0 + ivec2(i, j), ivec2(0), ivec2(srcsz) - ivec2(1));
                c[j][i] = texelFetch(tex, q, 0).rgb;
                l[j][i] = lum(c[j][i]);
            }
        /* Sobel-lite gradient energy per direction */
        float gx = abs(l[0][2]-l[0][0]) + 2.0*abs(l[1][2]-l[1][0]) + abs(l[2][2]-l[2][0]);
        float gy = abs(l[2][0]-l[0][0]) + 2.0*abs(l[2][1]-l[0][1]) + abs(l[2][2]-l[0][2]);
        float gd = abs(l[0][0]-l[2][2]) + abs(l[0][2]-l[2][0]);
        vec2 f = fract(px);
        vec3 nearest = c[1][1];
        /* interpolate ALONG the dominant edge, never across it */
        vec3 vres = mix(mix(c[0][1], c[1][1], f.y), mix(c[1][1], c[2][1], f.y), 0.5); /* horizontal edge */
        vec3 hres = mix(mix(c[1][0], c[1][1], f.x), mix(c[1][1], c[1][2], f.x), 0.5); /* vertical edge */
        vec3 dres = 0.5 * (mix(c[0][0], c[2][2], (f.x + f.y) * 0.5) +
                           mix(c[2][0], c[0][2], (f.x + 1.0 - f.y) * 0.5));           /* diagonals */
        float thr = 0.08;
        float wh = smoothstep(thr, 4.0*thr, gy - gx);
        float wv = smoothstep(thr, 4.0*thr, gx - gy);
        float wd = pc.dlss >= 2 ? smoothstep(thr, 4.0*thr, gd - max(gx, gy)) : 0.0;
        float wsum = wh + wv + wd;
        col = wsum > 0.0 ? mix(nearest, (wh*vres + wv*hres + wd*dres) / wsum, min(wsum, 1.0))
                         : nearest;
        /* contrast-adaptive sharpen (RCAS-style): unsharp masked by the local
           min/max so flat areas and overshoot are left alone */
        if (pc.sharp > 0.0) {
            vec3 blur = (c[0][1] + c[1][0] + c[1][2] + c[2][1]) * 0.25;
            vec3 mn = min(min(min(c[0][0], c[0][1]), min(c[0][2], c[1][0])),
                          min(min(c[1][2], c[2][0]), min(c[2][1], c[2][2])));
            vec3 mx = max(max(max(c[0][0], c[0][1]), max(c[0][2], c[1][0])),
                          max(max(c[1][2], c[2][0]), max(c[2][1], c[2][2])));
            col = clamp(col + pc.sharp * (col - blur), mn, mx);
        }
    }
    outc = vec4(col, 1.0);
    if (pc.scanlines == 1) {
        // darken every other source line (224 lines scaled)
        float sy = uv.y * 224.0;
        if (fract(sy) > 0.5) outc.rgb *= 0.65;
    }
}
