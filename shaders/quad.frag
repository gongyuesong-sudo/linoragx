#version 450
/* Pixel-art enhancement.
 *  enhance 0 = plain nearest (legacy)
 *  enhance 1 = generalized Scale2x (hard-edge corner fill)
 *  enhance 2 = xBR level 2 curve smoothing (DEFAULT)
 *
 * enhance 2 is a port of the pattern-recognition core of Hyllian's
 * xbr-lv2.glsl from libretro/glsl-shaders (MIT licensed), adapted to a
 * single Vulkan pass: neighborhood via texelFetch, and the sub-pixel grid
 * delta derived from screen-space derivatives so it works at any
 * (even non-integer) scale ratio. Curves and diagonals are cut with
 * straight-line inequations instead of staircase steps.
 *
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

/* ---------- enhance 1: generalized Scale2x ---------- */
vec3 scale2x(vec2 uvc)
{
    vec2 p = uvc * vec2(textureSize(tex, 0)) - 0.5;
    vec2 fl = floor(p);
    ivec2 c = ivec2(fl);
    vec2 f = p - fl;
    vec3 A = px(c + ivec2(-1,-1)).rgb, B = px(c + ivec2(0,-1)).rgb, C = px(c + ivec2(1,-1)).rgb;
    vec3 D = px(c + ivec2(-1, 0)).rgb, E = px(c).rgb,              F = px(c + ivec2(1, 0)).rgb;
    vec3 G = px(c + ivec2(-1, 1)).rgb, H = px(c + ivec2(0, 1)).rgb, I = px(c + ivec2(1, 1)).rgb;
    vec3 E0 = (D == A && A != C && D != G) ? A : E;
    vec3 E1 = (B == C && A != C && B != F) ? C : E;
    vec3 E2 = (D == G && D != I && G != H) ? G : E;
    vec3 E3 = (H == F && D != I && H != F) ? F : E;
    return mix(mix(E0, E1, f.x), mix(E2, E3, f.x), f.y);
}

/* ---------- enhance 2: xBR-lv2 (ported from Hyllian's xbr-lv2.glsl, MIT) ---------- */
const vec3 rgbw = vec3(14.352, 28.176, 5.472);
const vec4 eq_threshold = vec4(15.0);
const float lv2_cf = 2.0;

const vec4 Ao = vec4( 1.0, -1.0, -1.0, 1.0 );
const vec4 Bo = vec4( 1.0,  1.0, -1.0,-1.0 );
const vec4 Co = vec4( 1.5,  0.5, -0.5, 0.5 );
const vec4 Ax = vec4( 1.0, -1.0, -1.0, 1.0 );
const vec4 Bx = vec4( 0.5,  2.0, -0.5,-2.0 );
const vec4 Cx = vec4( 1.0,  1.0, -0.5, 0.0 );
const vec4 Ay = vec4( 1.0, -1.0, -1.0, 1.0 );
const vec4 By = vec4( 2.0,  0.5, -2.0,-0.5 );
const vec4 Cy = vec4( 2.0,  0.0, -1.0, 0.5 );
const vec4 Ci = vec4(0.25, 0.25, 0.25, 0.25);

vec4 df(vec4 A, vec4 B) { return vec4(abs(A - B)); }
vec4 diff(vec4 A, vec4 B) { return vec4(notEqual(A, B)); }
vec4 eq(vec4 A, vec4 B) { return step(df(A, B), eq_threshold); }
vec4 neq(vec4 A, vec4 B) { return vec4(1.0) - eq(A, B); }
vec4 wd(vec4 a, vec4 b, vec4 c, vec4 d, vec4 e, vec4 f, vec4 g, vec4 h)
{
    return (df(a,b) + df(a,c) + df(d,e) + df(d,f) + 4.0*df(g,h));
}
float c_df(vec3 c1, vec3 c2) { vec3 d = abs(c1 - c2); return d.r + d.g + d.b; }

vec3 xbr(vec2 uvc)
{
    vec2 sp = uvc * vec2(textureSize(tex, 0));
    ivec2 c0 = ivec2(floor(sp));
    vec2 fp = fract(sp);

    vec3 A1 = px(c0 + ivec2(-1,-2)).rgb, B1 = px(c0 + ivec2(0,-2)).rgb, C1 = px(c0 + ivec2(1,-2)).rgb;
    vec3 A  = px(c0 + ivec2(-1,-1)).rgb, B  = px(c0 + ivec2(0,-1)).rgb, C  = px(c0 + ivec2(1,-1)).rgb;
    vec3 D  = px(c0 + ivec2(-1, 0)).rgb, E  = px(c0).rgb,                F  = px(c0 + ivec2(1, 0)).rgb;
    vec3 G  = px(c0 + ivec2(-1, 1)).rgb, H  = px(c0 + ivec2(0, 1)).rgb,  I  = px(c0 + ivec2(1, 1)).rgb;
    vec3 G5 = px(c0 + ivec2(-1, 2)).rgb, H5 = px(c0 + ivec2(0, 2)).rgb,  I5 = px(c0 + ivec2(1, 2)).rgb;
    vec3 A0 = px(c0 + ivec2(-2,-1)).rgb, D0 = px(c0 + ivec2(-2, 0)).rgb, G0 = px(c0 + ivec2(-2, 1)).rgb;
    vec3 C4 = px(c0 + ivec2( 2,-1)).rgb, F4 = px(c0 + ivec2( 2, 0)).rgb, I4 = px(c0 + ivec2( 2, 1)).rgb;

    vec4 b = vec4(dot(B,rgbw), dot(D,rgbw), dot(H,rgbw), dot(F,rgbw));
    vec4 c = vec4(dot(C,rgbw), dot(A,rgbw), dot(G,rgbw), dot(I,rgbw));
    vec4 d = b.yzwx;
    vec4 e = vec4(dot(E,rgbw));
    vec4 f = b.wxyz;
    vec4 g = c.zwxy;
    vec4 h = b.zwxy;
    vec4 i = c.wxyz;
    vec4 i4 = vec4(dot(I4,rgbw), dot(C1,rgbw), dot(A0,rgbw), dot(G5,rgbw));
    vec4 i5 = vec4(dot(I5,rgbw), dot(C4,rgbw), dot(A1,rgbw), dot(G0,rgbw));
    vec4 h5 = vec4(dot(H5,rgbw), dot(F4,rgbw), dot(B1,rgbw), dot(D0,rgbw));
    vec4 f4 = h5.yzwx;

    /* texels per output pixel, from screen derivatives: works at any scale */
    vec2 srcsz = vec2(textureSize(tex, 0));
    float delta_s = (abs(dFdx(uvc.x)) + abs(dFdy(uvc.y))) * 0.25 * (srcsz.x + srcsz.y);
    vec4 delta   = vec4(delta_s);
    vec4 delta_l = vec4(0.5*delta_s, delta_s, 0.5*delta_s, delta_s);
    vec4 delta_u = delta_l.yxwz;

    vec4 fx   = (Ao*fp.y + Bo*fp.x);
    vec4 fx_l = (Ax*fp.y + Bx*fp.x);
    vec4 fx_u = (Ay*fp.y + By*fp.x);

    vec4 irlv1, irlv0;
    irlv1 = irlv0 = diff(e,f) * diff(e,h);
    /* CORNER_C variant (upstream default, with SMOOTH_TIPS) */
    irlv1 = irlv0 * ( neq(f,b) * neq(f,c) + neq(h,d) * neq(h,g)
                    + eq(e,i) * (neq(f,f4) * neq(f,i4) + neq(h,h5) * neq(h,i5))
                    + eq(e,g) + eq(e,c) );

    vec4 irlv2l = diff(e,g) * diff(d,g);
    vec4 irlv2u = diff(e,c) * diff(b,c);

    vec4 fx45i = clamp((fx   + delta   - Co - Ci) / (2.0*delta  ), 0.0, 1.0);
    vec4 fx45  = clamp((fx   + delta   - Co     ) / (2.0*delta  ), 0.0, 1.0);
    vec4 fx30  = clamp((fx_l + delta_l - Cx     ) / (2.0*delta_l), 0.0, 1.0);
    vec4 fx60  = clamp((fx_u + delta_u - Cy     ) / (2.0*delta_u), 0.0, 1.0);

    vec4 wd1 = wd( e, c,  g, i, h5, f4, h, f);
    vec4 wd2 = wd( h, d, i5, f, i4,  b, e, i);

    vec4 edri  = step(wd1, wd2) * irlv0;
    vec4 edr   = step(wd1 + vec4(0.1), wd2) * step(vec4(0.5), irlv1);
    vec4 edr_l = step( lv2_cf*df(f,g), df(h,c) ) * irlv2l * edr;
    vec4 edr_u = step( lv2_cf*df(h,c), df(f,g) ) * irlv2u * edr;

    fx45  = edr   * fx45;
    fx30  = edr_l * fx30;
    fx60  = edr_u * fx60;
    fx45i = edri  * fx45i;

    vec4 pxv = step(df(e,f), df(e,h));
    vec4 maximos = max(max(fx30, fx60), max(fx45, fx45i));

    vec3 res1 = E;
    res1 = mix(res1, mix(H, F, pxv.x), maximos.x);
    res1 = mix(res1, mix(B, D, pxv.z), maximos.z);
    vec3 res2 = E;
    res2 = mix(res2, mix(F, B, pxv.y), maximos.y);
    res2 = mix(res2, mix(D, H, pxv.w), maximos.w);
    return mix(res1, res2, step(c_df(E, res1), c_df(E, res2)));
}

void main()
{
    vec3 col;
    if (pc.enhance == 0)      col = texture(tex, uv).rgb;
    else if (pc.enhance == 1) col = scale2x(uv);
    else                      col = xbr(uv);
    outc = vec4(col, 1.0);
    if (pc.scanlines == 1) {
        // darken every other source line (224 lines scaled)
        float sy = uv.y * 224.0;
        if (fract(sy) > 0.5) outc.rgb *= 0.65;
    }
}
