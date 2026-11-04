#version 450
/* Pixel-art enhancement.
 *  enhance 0 = plain nearest (legacy)
 *  enhance 1 = generalized Scale2x (hard-edge corner fill)
 *  enhance 2 = xBR level 2 curve smoothing
 *  enhance 3 = xBRZ freescale (DEFAULT)
 *
 * enhance 2 is a port of the pattern-recognition core of Hyllian's
 * xbr-lv2.glsl from libretro/glsl-shaders (MIT licensed), adapted to a
 * single Vulkan pass: neighborhood via texelFetch, and the sub-pixel grid
 * delta derived from screen-space derivatives so it works at any
 * (even non-integer) scale ratio. Curves and diagonals are cut with
 * straight-line inequations instead of staircase steps.
 *
 * enhance 3 is a port of xbrz-freescale.slang from libretro/slang-shaders
 * (edge-smoothing/xbrz/shaders), which implements Zenju's xBRZ: YCbCr
 * color distance, dominant-gradient corner preprocessing and line-angle
 * aware corner blending. That source is GPL-3.0 (xBRZ (C) Zenju, vertex
 * mapping (C) Hyllian MIT); note this shader file is therefore GPL-3.0.
 * Scale ratio likewise derived from screen-space derivatives.
 *
 * Letterbox is handled C-side (viewport fits 320:224, bars = clear color),
 * so uv here always spans exactly the game image. */
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outc;
layout(set = 0, binding = 0) uniform sampler2D tex;
layout(push_constant) uniform PC {
    int scanlines;
    int enhance;
    int dstw;
    int dsth;
    int crt;
} pc;

/* CRT emulation (aperture-grille style): gaussian scanline beam + RGB
 * phosphor stripes + persistence gain. Modulation depth (~36%) matches
 * cgwg's crt-aperture look. fc = viewport pixel coord, dst = viewport
 * size, src = source signal size. */
vec3 crt_emulate(vec3 c, vec2 fc, vec2 dst, vec2 src)
{
    float rows = dst.y / src.y;
    float sy = fract(fc.y / rows) - 0.5;
    c *= clamp(exp(-(sy * sy) / 0.16), 0.64, 1.0);
    vec3 stripe = vec3(0.70);
    stripe[int(mod(floor(fc.x), 3.0))] = 1.30;
    return c * stripe * 1.5;
}

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

/* ---------- enhance 3: xBRZ (ported from xbrz-freescale.slang, GPL-3.0) ---------- */
const float XZ_EQ_TOL = 30.0 / 255.0;
const float XZ_STEEP = 2.2;
const float XZ_DOMINANT = 3.6;

float distYCbCr(vec3 a, vec3 b)
{
    const vec3 w = vec3(0.2627, 0.6780, 0.0593);
    float sB = 0.5 / (1.0 - w.b);
    float sR = 0.5 / (1.0 - w.r);
    vec3 d = a - b;
    float Y = dot(d, w);
    float Cb = sB * (d.b - Y);
    float Cr = sR * (d.r - Y);
    return sqrt(Y * Y + Cb * Cb + Cr * Cr);
}
bool pixEq(vec3 a, vec3 b) { return distYCbCr(a, b) < XZ_EQ_TOL; }

float left_ratio(vec2 center, vec2 origin, vec2 dir, vec2 scale)
{
    vec2 P0 = center - origin;
    vec2 proj = dir * (dot(P0, dir) / dot(dir, dir));
    vec2 distv = P0 - proj;
    vec2 orth = vec2(-dir.y, dir.x);
    float side = sign(dot(P0, orth));
    float v = side * length(distv * scale);
    return smoothstep(-sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, v);
}

vec3 xbrz(vec2 uvc)
{
    vec2 srcsz = vec2(textureSize(tex, 0));
    vec2 sp = uvc * srcsz;
    ivec2 c0 = ivec2(floor(sp));
    vec2 pos = fract(sp) - 0.5;
    /* output pixels per source texel, from screen derivatives */
    vec2 scale = 1.0 / max(abs(vec2(dFdx(uvc.x), dFdy(uvc.y))) * srcsz, vec2(1e-6));

    vec3 A = px(c0 + ivec2(-1,-1)).rgb, B = px(c0 + ivec2(0,-1)).rgb, C = px(c0 + ivec2(1,-1)).rgb;
    vec3 D = px(c0 + ivec2(-1, 0)).rgb, E = px(c0).rgb,                F = px(c0 + ivec2(1, 0)).rgb;
    vec3 G = px(c0 + ivec2(-1, 1)).rgb, H = px(c0 + ivec2(0, 1)).rgb,  I = px(c0 + ivec2(1, 1)).rgb;

    ivec4 blendResult = ivec4(0, 0, 0, 0);

    /* preprocess corner z (lower-right) */
    if (!((E == F && H == I) || (E == H && F == I))) {
        float dHF = distYCbCr(G, E) + distYCbCr(E, C)
                  + distYCbCr(px(c0 + ivec2(0, 2)).rgb, I) + distYCbCr(I, px(c0 + ivec2(2, 0)).rgb)
                  + 4.0 * distYCbCr(H, F);
        float dEI = distYCbCr(D, H) + distYCbCr(H, px(c0 + ivec2(1, 2)).rgb)
                  + distYCbCr(B, F) + distYCbCr(F, px(c0 + ivec2(2, 1)).rgb)
                  + 4.0 * distYCbCr(E, I);
        bool dom = (XZ_DOMINANT * dHF) < dEI;
        blendResult.z = ((dHF < dEI) && E != F && E != H) ? (dom ? 2 : 1) : 0;
    }
    /* preprocess corner w (lower-left) */
    if (!((D == E && G == H) || (D == G && E == H))) {
        float dGE = distYCbCr(px(c0 + ivec2(-2, 1)).rgb, D) + distYCbCr(D, B)
                  + distYCbCr(px(c0 + ivec2(-1, 2)).rgb, H) + distYCbCr(H, F)
                  + 4.0 * distYCbCr(G, E);
        float dDH = distYCbCr(px(c0 + ivec2(-2, 0)).rgb, G) + distYCbCr(G, px(c0 + ivec2(0, 2)).rgb)
                  + distYCbCr(A, E) + distYCbCr(E, I)
                  + 4.0 * distYCbCr(D, H);
        bool dom = (XZ_DOMINANT * dDH) < dGE;
        blendResult.w = ((dGE > dDH) && E != D && E != H) ? (dom ? 2 : 1) : 0;
    }
    /* preprocess corner y (upper-right) */
    if (!((B == C && E == F) || (B == E && C == F))) {
        float dEC = distYCbCr(D, B) + distYCbCr(B, px(c0 + ivec2(1, -2)).rgb)
                  + distYCbCr(H, F) + distYCbCr(F, px(c0 + ivec2(2, -1)).rgb)
                  + 4.0 * distYCbCr(E, C);
        float dBF = distYCbCr(A, E) + distYCbCr(E, I)
                  + distYCbCr(px(c0 + ivec2(0, -2)).rgb, C) + distYCbCr(C, px(c0 + ivec2(2, 0)).rgb)
                  + 4.0 * distYCbCr(B, F);
        bool dom = (XZ_DOMINANT * dBF) < dEC;
        blendResult.y = ((dEC > dBF) && E != B && E != F) ? (dom ? 2 : 1) : 0;
    }
    /* preprocess corner x (upper-left) */
    if (!((A == B && D == E) || (A == D && B == E))) {
        float dDB = distYCbCr(px(c0 + ivec2(-2, 0)).rgb, A) + distYCbCr(A, px(c0 + ivec2(0, -2)).rgb)
                  + distYCbCr(G, E) + distYCbCr(E, C)
                  + 4.0 * distYCbCr(D, B);
        float dAE = distYCbCr(px(c0 + ivec2(-2, -1)).rgb, D) + distYCbCr(D, H)
                  + distYCbCr(px(c0 + ivec2(-1, -2)).rgb, B) + distYCbCr(B, F)
                  + 4.0 * distYCbCr(A, E);
        bool dom = (XZ_DOMINANT * dDB) < dAE;
        blendResult.x = ((dDB < dAE) && E != D && E != B) ? (dom ? 2 : 1) : 0;
    }

    vec3 res = E;
    const float r2 = 1.0 / sqrt(2.0);

    /* blend corner z */
    if (blendResult.z != 0) {
        float dFG = distYCbCr(F, G);
        float dHC = distYCbCr(H, C);
        bool doLineBlend = (blendResult.z == 2 ||
            !((blendResult.y != 0 && !pixEq(E, G)) || (blendResult.w != 0 && !pixEq(E, C)) ||
              (pixEq(G, H) && pixEq(H, I) && pixEq(I, F) && pixEq(F, C) && !pixEq(E, I))));
        vec2 origin = vec2(0.0, r2);
        vec2 direction = vec2(1.0, -1.0);
        if (doLineBlend) {
            bool shallow = (XZ_STEEP * dFG <= dHC) && E != G && D != G;
            bool steep   = (XZ_STEEP * dHC <= dFG) && E != C && B != C;
            origin = shallow ? vec2(0.0, 0.25) : vec2(0.0, 0.5);
            direction.x += shallow ? 1.0 : 0.0;
            direction.y -= steep ? 1.0 : 0.0;
        }
        vec3 blendPix = mix(H, F, step(distYCbCr(E, F), distYCbCr(E, H)));
        res = mix(res, blendPix, left_ratio(pos, origin, direction, scale));
    }
    /* blend corner w */
    if (blendResult.w != 0) {
        float dHA = distYCbCr(H, A);
        float dDI = distYCbCr(D, I);
        bool doLineBlend = (blendResult.w == 2 ||
            !((blendResult.z != 0 && !pixEq(E, A)) || (blendResult.x != 0 && !pixEq(E, I)) ||
              (pixEq(A, D) && pixEq(D, G) && pixEq(G, H) && pixEq(H, I) && !pixEq(E, G))));
        vec2 origin = vec2(-r2, 0.0);
        vec2 direction = vec2(1.0, 1.0);
        if (doLineBlend) {
            bool shallow = (XZ_STEEP * dHA <= dDI) && E != A && B != A;
            bool steep   = (XZ_STEEP * dDI <= dHA) && E != I && F != I;
            origin = shallow ? vec2(-0.25, 0.0) : vec2(-0.5, 0.0);
            direction.y += shallow ? 1.0 : 0.0;
            direction.x += steep ? 1.0 : 0.0;
        }
        vec3 blendPix = mix(H, D, step(distYCbCr(E, D), distYCbCr(E, H)));
        res = mix(res, blendPix, left_ratio(pos, origin, direction, scale));
    }
    /* blend corner y */
    if (blendResult.y != 0) {
        float dBI = distYCbCr(B, I);
        float dFA = distYCbCr(F, A);
        bool doLineBlend = (blendResult.y == 2 ||
            !((blendResult.x != 0 && !pixEq(E, I)) || (blendResult.z != 0 && !pixEq(E, A)) ||
              (pixEq(I, F) && pixEq(F, C) && pixEq(C, B) && pixEq(B, A) && !pixEq(E, C))));
        vec2 origin = vec2(r2, 0.0);
        vec2 direction = vec2(-1.0, -1.0);
        if (doLineBlend) {
            bool shallow = (XZ_STEEP * dBI <= dFA) && E != I && H != I;
            bool steep   = (XZ_STEEP * dFA <= dBI) && E != A && D != A;
            origin = shallow ? vec2(0.25, 0.0) : vec2(0.5, 0.0);
            direction.y -= shallow ? 1.0 : 0.0;
            direction.x -= steep ? 1.0 : 0.0;
        }
        vec3 blendPix = mix(F, B, step(distYCbCr(E, B), distYCbCr(E, F)));
        res = mix(res, blendPix, left_ratio(pos, origin, direction, scale));
    }
    /* blend corner x */
    if (blendResult.x != 0) {
        float dDC = distYCbCr(D, C);
        float dBG = distYCbCr(B, G);
        bool doLineBlend = (blendResult.x == 2 ||
            !((blendResult.w != 0 && !pixEq(E, C)) || (blendResult.y != 0 && !pixEq(E, G)) ||
              (pixEq(C, B) && pixEq(B, A) && pixEq(A, D) && pixEq(D, G) && !pixEq(E, A))));
        vec2 origin = vec2(0.0, -r2);
        vec2 direction = vec2(-1.0, 1.0);
        if (doLineBlend) {
            bool shallow = (XZ_STEEP * dDC <= dBG) && E != C && F != C;
            bool steep   = (XZ_STEEP * dBG <= dDC) && E != G && H != G;
            origin = shallow ? vec2(0.0, -0.25) : vec2(0.0, -0.5);
            direction.x -= shallow ? 1.0 : 0.0;
            direction.y += steep ? 1.0 : 0.0;
        }
        vec3 blendPix = mix(D, B, step(distYCbCr(E, B), distYCbCr(E, D)));
        res = mix(res, blendPix, left_ratio(pos, origin, direction, scale));
    }
    return res;
}

void main()
{
    vec3 col;
    if (pc.enhance == 0)      col = texture(tex, uv).rgb;
    else if (pc.enhance == 1) col = scale2x(uv);
    else if (pc.enhance == 2) col = xbr(uv);
    else                      col = xbrz(uv);
    outc = vec4(col, 1.0);
    if (pc.crt == 1) {
        outc.rgb = crt_emulate(outc.rgb, gl_FragCoord.xy,
                              vec2(float(pc.dstw), float(pc.dsth)),
                              vec2(textureSize(tex, 0)));
    } else if (pc.scanlines == 1) {
        // darken every other source line (224 lines scaled)
        float sy = uv.y * 224.0;
        if (fract(sy) > 0.5) outc.rgb *= 0.65;
    }
}
