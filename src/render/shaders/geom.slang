// One pass, two outputs: Lambert-shaded colour (RGBA8) and linear depth in metres (R32F, = view-space z along the
// optical axis). The CPU builds the matrices in double relative to the camera (see renderer.cpp), so everything
// here is small-magnitude float.
struct Push {
  float4 mvp[4];  // columns of clip-from-geom (camera-relative translation already folded in); clip.w = view z
  float4 rot[3];  // columns of world-from-geom rotation (normals)
  float4 color;
};
[[vk::push_constant]] ConstantBuffer<Push> pc;

struct VsIn {
  float3 pos : POSITION;
  float3 nrm : NORMAL;
};
struct VsOut {
  float4 clip : SV_Position;
  float3 n : NORMAL;
  float z : TEXCOORD0;
};
struct FsOut {
  float4 color : SV_Target0;
  float depth : SV_Target1;
};

[shader("vertex")]
VsOut vsMain(VsIn i) {
  VsOut o;
  o.clip = pc.mvp[0] * i.pos.x + pc.mvp[1] * i.pos.y + pc.mvp[2] * i.pos.z + pc.mvp[3];
  o.n = pc.rot[0].xyz * i.nrm.x + pc.rot[1].xyz * i.nrm.y + pc.rot[2].xyz * i.nrm.z;
  o.z = o.clip.w;
  return o;
}

[shader("fragment")]
FsOut fsMain(VsOut i) {
  // One directional light, fixed in the world frame (pointing roughly down from +x,+y).
  const float3 L = normalize(float3(0.4, 0.3, 0.85));
  const float lambert = max(dot(normalize(i.n), L), 0.0);
  FsOut o;
  o.color = float4(pc.color.rgb * (0.25 + 0.75 * lambert), 1.0);
  o.depth = i.z;
  return o;
}
