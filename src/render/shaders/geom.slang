// The renderer's shaders (entry points; renderer.cpp builds one pipeline per use):
//   vsMain/fsMain   geometry: sun + hemisphere light, Blinn-Phong/Fresnel specular, PCF sun shadow, distance haze, sRGB out
//   vsMain/fsLin    the same geometry into the R32F linear-depth target (view-space z in metres, exact)
//   vsShadow        depth only, from the sun
//   vsSky/fsSky     full-screen procedural sky (gradient, warm horizon, sun, noise clouds)
//   vsEdge/fsEdge   CAD-style crease lines: each segment is a screen-space quad of constant pixel width
// The CPU builds everything camera-relative in double (see renderer.cpp); here all values are small-magnitude floats.
struct Scene {
  float4 view_r, view_u, view_f;  // camera basis rows (xyz)
  float4 proj;                    // sx, sy, A, B: clip = (sx*x, sy*y, A*z+B, z)
  float4 lcol[4];                 // columns of light-clip-from-camera-relative (xyz; the last is the translation)
  float4 sun_dir;                 // xyz: toward the sun; w: glare strength
  float4 sun_col;                 // rgb: sun radiance
  float4 sky_top, sky_mid, sky_hor, ground;  // rgb gradient stops; ground = lower-hemisphere ambient
  float4 ground_col;              // rgb: ground-plane albedo
  float4 fog;                     // x: haze density (1/m), y: style (0 outdoor, 1 studio), z: edge width (px), w: ambient scale
  float4 vp;                      // viewport w, h; shadows enabled; shadow texel size in metres
  float4 camf;                    // camera position mod 10 (xy), z; 1/shadow-map size in w
};
struct Draw {
  float4 rot[3];  // columns of world-from-geom rotation
  float4 trel;    // xyz: camera-relative translation (double-subtracted on the CPU)
  float4 color;
  float4 mat;     // specular, shininess, metallic, roughness (metallic / roughness < 0: unset)
  float4 flags;   // x: 1 = ground plane
};
[[vk::binding(0, 0)]] ConstantBuffer<Scene> sc;
[[vk::binding(1, 0)]] StructuredBuffer<Draw> draws;
[[vk::binding(2, 0)]] Texture2D<float> shadowTex;
[[vk::binding(3, 0)]] SamplerComparisonState shadowSmp;
struct Push {
  uint draw;
};
[[vk::push_constant]] ConstantBuffer<Push> pc;

static const float kPi = 3.14159265;

float3 to_lin(float3 c) { return pow(max(c, 0.0), 2.2); }
float3 to_srgb(float3 c) {
  c = saturate(c);
  return select(c <= 0.0031308, c * 12.92, 1.055 * pow(c, 1.0 / 2.4) - 0.055);
}

float4 clip_of(float3 rel) {
  const float3 v = float3(dot(sc.view_r.xyz, rel), dot(sc.view_u.xyz, rel), dot(sc.view_f.xyz, rel));
  return float4(sc.proj.x * v.x, sc.proj.y * v.y, sc.proj.z * v.z + sc.proj.w, v.z);
}

// ---- geometry --------------------------------------------------------------------------------------------------------
struct VsIn {
  float3 pos : POSITION;
  float3 nrm : NORMAL;
};
struct VsOut {
  float4 clip : SV_Position;
  float3 rel : TEXCOORD0;
  float3 n : NORMAL;
};

[shader("vertex")]
VsOut vsMain(VsIn i) {
  const Draw d = draws[pc.draw];
  VsOut o;
  o.rel = d.rot[0].xyz * i.pos.x + d.rot[1].xyz * i.pos.y + d.rot[2].xyz * i.pos.z + d.trel.xyz;
  o.n = d.rot[0].xyz * i.nrm.x + d.rot[1].xyz * i.nrm.y + d.rot[2].xyz * i.nrm.z;
  o.clip = clip_of(o.rel);
  return o;
}

[shader("vertex")]
float4 vsShadow(VsIn i) : SV_Position {
  const Draw d = draws[pc.draw];
  const float3 rel = d.rot[0].xyz * i.pos.x + d.rot[1].xyz * i.pos.y + d.rot[2].xyz * i.pos.z + d.trel.xyz;
  return float4(sc.lcol[0].xyz * rel.x + sc.lcol[1].xyz * rel.y + sc.lcol[2].xyz * rel.z + sc.lcol[3].xyz, 1);
}

[shader("fragment")]
float fsLin(VsOut i) : SV_Target0 { return dot(sc.view_f.xyz, i.rel); }

float shadow_vis(float3 rel, float3 n) {
  if (sc.vp.z < 0.5) return 1.0;
  const float3 p = rel + n * (sc.vp.w * 1.5);  // normal offset: kills most acne without a big depth bias
  const float3 s = sc.lcol[0].xyz * p.x + sc.lcol[1].xyz * p.y + sc.lcol[2].xyz * p.z + sc.lcol[3].xyz;
  const float2 uv = s.xy * 0.5 + 0.5;
  if (any(uv < 0.0) || any(uv > 1.0) || s.z > 1.0) return 1.0;  // outside the fitted map: lit (not the border's "all shadow")
  const float px = sc.camf.w;
  float sum = 0;
  [unroll] for (int y = -1; y <= 1; ++y)
    [unroll] for (int x = -1; x <= 1; ++x) sum += shadowTex.SampleCmpLevelZero(shadowSmp, uv + float2(x, y) * px, s.z - 0.0005);
  return sum / 9.0;
}

// ---- sky -------------------------------------------------------------------------------------------------------------
float hash21(float2 p) {
  p = frac(p * float2(123.34, 456.21));
  p += dot(p, p + 45.32);
  return frac(p.x * p.y);
}
float vnoise(float2 p) {
  const float2 i = floor(p), f = frac(p), u = f * f * (3.0 - 2.0 * f);
  return lerp(lerp(hash21(i), hash21(i + float2(1, 0)), u.x), lerp(hash21(i + float2(0, 1)), hash21(i + float2(1, 1)), u.x), u.y);
}
float fbm(float2 p) {
  float a = 0.5, s = 0;
  [unroll] for (int k = 0; k < 5; ++k) {
    s += a * vnoise(p);
    p = p * 2.03 + 17.0;
    a *= 0.5;
  }
  return s;
}

// Sky colour in a direction, without clouds / sun disc (also the reflection probe of shiny surfaces).
float3 sky_base(float3 dir) {
  const float e = dir.z;
  float3 c = lerp(sc.sky_hor.rgb, sc.sky_mid.rgb, smoothstep(0.0, 0.28, e));
  c = lerp(c, sc.sky_top.rgb, smoothstep(0.18, 0.95, e));
  return lerp(sc.sky_hor.rgb, c, smoothstep(-0.08, 0.0, e));  // below the horizon: the haze colour
}

[shader("vertex")]
float4 vsSky(uint vid : SV_VertexID) : SV_Position {
  const float2 p = float2(vid == 1 ? 3.0 : -1.0, vid == 2 ? 3.0 : -1.0);
  return float4(p, 1, 1);
}

[shader("fragment")]
float4 fsSky(float4 frag : SV_Position) : SV_Target0 {
  const float2 ndc = frag.xy / sc.vp.xy * 2.0 - 1.0;
  const float3 dir = normalize(sc.view_f.xyz + (ndc.x / sc.proj.x) * sc.view_r.xyz + (ndc.y / sc.proj.y) * sc.view_u.xyz);
  float3 c = sky_base(dir);
  if (sc.fog.y < 0.5) {
    const float3 L = sc.sun_dir.xyz;
    const float sd = saturate(dot(dir, L));
    c += float3(1.0, 0.72, 0.42) * (0.55 * pow(sd, 6.0) + 0.5 * pow(sd, 48.0)) * sc.sun_dir.w;  // warm glare
    c += float3(1.0, 0.95, 0.85) * smoothstep(0.9993, 0.9998, sd) * 4.0;
    if (dir.z > 0.0) {  // clouds: fbm on a plane overhead, fading into the haze toward the horizon
      const float2 uv = dir.xy / (dir.z + 0.12) * 1.3 + sc.camf.xy * 0.004;
      const float n = fbm(uv);
      const float cov = smoothstep(0.50, 0.80, n) * smoothstep(0.0, 0.16, dir.z);
      const float lit = saturate(0.55 + (n - fbm(uv + L.xy * 0.12)) * 3.0);
      float3 cc = lerp(float3(0.66, 0.70, 0.80), float3(1.0, 0.97, 0.93), lit);
      cc = lerp(sc.sky_hor.rgb, cc, smoothstep(0.0, 0.4, dir.z));  // low clouds take the horizon's warm glow
      c = lerp(c, cc, cov * 0.92);
    }
  }
  return float4(to_srgb(c), 1);
}

// ---- lit surfaces ----------------------------------------------------------------------------------------------------
[shader("fragment")]
float4 fsMain(VsOut i) : SV_Target0 {
  const Draw d = draws[pc.draw];
  const float3 V = -normalize(i.rel);
  float3 N = normalize(i.n);
  const bool ground = d.flags.x > 0.5;
  if (ground && N.z * V.z < 0) N = -N;
  float3 albedo = ground ? sc.ground_col.rgb : to_lin(d.color.rgb);
  if (ground && sc.fog.y > 0.5) {  // studio floor: 1 m / 0.1 m grid, fading out
    const float2 w = i.rel.xy + sc.camf.xy;
    const float dist = length(i.rel);
    const float2 fw = max(fwidth(w), 1e-5);
    const float2 g1 = abs(frac(w - 0.5) - 0.5) / fw, g2 = abs(frac(w * 10.0 - 0.5) - 0.5) / (fw * 10.0);
    const float l1 = 1.0 - saturate(min(g1.x, g1.y) - 0.5), l2 = (1.0 - saturate(min(g2.x, g2.y) - 0.5)) * exp(-dist * 0.35);
    // 1 cm lines: only close to the camera, and only while a pixel is much smaller than the pitch (benchtop-scale designs)
    const float2 g3 = abs(frac(w * 100.0 - 0.5) - 0.5) / (fw * 100.0);
    const float l3 = (1.0 - saturate(min(g3.x, g3.y) - 0.5)) * exp(-dist * 3.0) * saturate(1.0 - max(fw.x, fw.y) * 80.0);
    albedo = lerp(albedo, albedo * 0.45, saturate(max(max(l1 * 0.9, l2 * 0.35), l3 * 0.16)));
  }
  const float metal = d.mat.z >= 0 ? d.mat.z : 0.0;
  const float3 L = sc.sun_dir.xyz;
  const float ndl = saturate(dot(N, L));
  const float vis = ndl > 0 ? shadow_vis(i.rel, N) : 1.0;
  const float3 hemi = lerp(sc.ground.rgb, sc.sky_mid.rgb, N.z * 0.5 + 0.5) * sc.fog.w;
  float3 col = albedo * (1.0 - metal) * (sc.sun_col.rgb * ndl * vis + hemi);
  // specular: Blinn-Phong lobe (exponent from shininess or roughness) x Schlick Fresnel; metals tint it with the albedo
  float shin = d.mat.y * 128.0;
  if (d.mat.w >= 0) shin = clamp(2.0 / max(d.mat.w * d.mat.w * d.mat.w * d.mat.w, 1e-4) - 2.0, 1.0, 1024.0);
  const float f0 = d.mat.x * 0.08;
  const float3 F0 = lerp(float3(f0, f0, f0), albedo, metal);
  const float3 H = normalize(L + V);
  const float3 F = F0 + (1.0 - F0) * pow(1.0 - saturate(dot(V, H)), 5.0);
  col += F * pow(saturate(dot(N, H)), shin) * (shin + 8.0) / (8.0 * kPi) * ndl * vis * sc.sun_col.rgb;
  if (!ground) col += F0 * sky_base(reflect(-V, N)) * (0.35 + 0.65 * metal) * sc.fog.w;  // sky reflection
  // distance haze toward the horizon colour
  const float dist = length(i.rel);
  // outdoor: the ground melts into the horizon within ~100 m while objects (drones at range) stay readable
  const float k = sc.fog.y > 0.5 ? 1.0 : (ground ? 4.0 : 0.5);
  const float haze = 1.0 - exp(-dist * sc.fog.x * k);
  col = lerp(col, sc.sky_hor.rgb, saturate(haze));
  return float4(to_srgb(col), 1);
}

// ---- edges -----------------------------------------------------------------------------------------------------------
// Vertex layout reuse: pos = one end, nrm = the other end of the segment; vid % 6 picks the quad corner.
float4 biased(float3 rel) {  // clip position pulled slightly toward the camera so lines win the depth test on their own faces
  float4 c = clip_of(rel);
  const float zb = c.w * 0.997 - 0.0004;
  c.z = (sc.proj.z + sc.proj.w / zb) * c.w;
  return c;
}
[shader("vertex")]
float4 vsEdge(VsIn i, uint vid : SV_VertexID) : SV_Position {
  const Draw d = draws[pc.draw];
  const float3 a = d.rot[0].xyz * i.pos.x + d.rot[1].xyz * i.pos.y + d.rot[2].xyz * i.pos.z + d.trel.xyz;
  const float3 b = d.rot[0].xyz * i.nrm.x + d.rot[1].xyz * i.nrm.y + d.rot[2].xyz * i.nrm.z + d.trel.xyz;
  float4 ca = biased(a), cb = biased(b);
  const uint k = vid % 6;
  const bool at_b = (k == 1 || k == 2 || k == 4);
  const float side = (k == 2 || k == 4 || k == 5) ? 1.0 : -1.0;
  // a segment crossing the near plane is pulled to the plane (cheap clip: keeps its direction)
  const float nearw = 0.06;
  if (ca.w < nearw && cb.w < nearw) return float4(0, 0, -2, 1);
  if (ca.w < nearw) ca = lerp(ca, cb, (nearw - ca.w) / (cb.w - ca.w));
  if (cb.w < nearw) cb = lerp(cb, ca, (nearw - cb.w) / (ca.w - cb.w));
  const float2 sa = ca.xy / ca.w * sc.vp.xy * 0.5, sb = cb.xy / cb.w * sc.vp.xy * 0.5;
  float2 dir = sb - sa;
  dir = length(dir) > 1e-6 ? normalize(dir) : float2(1, 0);
  const float2 off = float2(-dir.y, dir.x) * side * sc.fog.z / sc.vp.xy;  // half width in px -> ndc: px * 2 / vp * 0.5
  float4 c = at_b ? cb : ca;
  c.xy += off * c.w;
  return c;
}
[shader("fragment")]
float4 fsEdge() : SV_Target0 { return float4(to_srgb(float3(0.012, 0.014, 0.02)), 1); }
