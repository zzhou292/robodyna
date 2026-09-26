// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <climits>
namespace tlfea::contact::radioss_type25::startup::detail {
namespace {
bool Output(tl::util::BoundedArenaLayout& arena,std::size_t p,std::size_t g,OutputLayout& out,TopologyPolicy policy,std::size_t raw,bool sides_only) noexcept {
  const auto refs=4*g;
  return arena.Append<Main>(g,out.mains) &&
      arena.Append<std::uint32_t>(g,out.expanded_to_primary) &&
      arena.Append<std::uint32_t>(p,out.primary_to_partner) &&
      (sides_only ||
       (arena.Append<StoredNormal>(4*g,out.normals) &&
        arena.Append<NormalReference>(refs,out.references) &&
        arena.Append<std::uint32_t>(refs+1,out.normal_offsets) &&
        arena.Append<std::uint32_t>(4*g,out.normal_mains))) &&
      (!role_policy::StoresRoles(policy) || arena.Append<ShellSideRole>(p,out.primary_roles)) &&
      (!role_policy::Mixed(policy) ||
       (arena.Append<PrimaryFaceIdentity>(p,out.primary_identities) &&
        arena.Append<PrimaryFaceIdentity>(raw,out.raw_origins) &&
        arena.Append<std::uint32_t>(raw,out.raw_origin_to_primary))) &&
      (!role_policy::Mixed(policy) || sides_only ||
       (arena.Append<PostGapmTopology>(1,out.post_gapm) &&
        arena.Append<PrimaryCornerPermutation>(p,out.post_corners) &&
        arena.Append<PreShellSolidSupport>(p,out.post_before) &&
        arena.Append<PostGapmMainSupport>(g,out.post_support)));
}
}
Report MakeLayout(std::size_t nodes,std::size_t p,Limits limits,Layout& output,TopologyPolicy policy,std::size_t shells,std::size_t raw,bool sides_only) noexcept {
  if(policy!=TopologyPolicy::ManifoldTwoSided && policy!=TopologyPolicy::NativeOrdinaryShell &&
      policy!=TopologyPolicy::NativeResolvedShellSides && policy!=TopologyPolicy::NativeMixedSurface)
    return {Status::UnsupportedProfile};
  const Limits hard;
  if (!nodes || !p) return {Status::InvalidInput};
  if (!limits.max_nodes || limits.max_nodes>hard.max_nodes || !limits.max_primary_faces ||
      limits.max_primary_faces>hard.max_primary_faces || !limits.max_output_bytes ||
      !limits.max_scratch_bytes || nodes>limits.max_nodes || p>limits.max_primary_faces ||
      p>std::size_t(INT_MAX)/8 || nodes>std::size_t(INT_MAX)) return {Status::ResourceLimit};
  if(role_policy::Mixed(policy)) {
    if(shells>p || raw<p)return {Status::InvalidInput};
    if(!limits.max_raw_origins || limits.max_raw_origins>hard.max_raw_origins || raw>limits.max_raw_origins)
      return {Status::ResourceLimit};
  }
  Layout next;
  const auto g=role_policy::Mixed(policy)?p+shells:2*p;
  tl::util::BoundedArenaLayout persistent(SIZE_MAX),scratch(SIZE_MAX),ready(SIZE_MAX);
  OutputLayout staging;
  if (!Output(persistent,p,g,next.output,policy,raw,sides_only) || !Output(scratch,p,g,staging,policy,raw,sides_only) ||
      !scratch.Append<Vector>(nodes,next.points) ||
      (!sides_only && !scratch.Append<Edge>(4*g,next.edges)) ||
      !scratch.Append<FaceKey>(p,next.face_keys) ||
      !scratch.Append<Identity>(nodes>p?nodes:p,next.identities) ||
      (!sides_only &&
       (!scratch.Append<int>(4*g,next.parents) || !scratch.Append<int>(4*g,next.tags) ||
        !scratch.Append<std::uint32_t>(nodes,next.node_references)))) return {Status::ResourceLimit};
  if(!sides_only && policy!=TopologyPolicy::ManifoldTwoSided &&
      (!scratch.Append<int>(g+4,next.candidate_ids) ||
       !scratch.Append<double>(g,next.candidate_angles) ||
       !scratch.Append<double>(g,next.candidate_sides))) return {Status::ResourceLimit};
  tl::util::ArenaRegion normal_region,reference_region;
  if (policy==TopologyPolicy::ManifoldTwoSided &&
      (!ready.Append<StoredNormal>(4*g,normal_region) ||
       !ready.Append<NormalReference>(4*g,reference_region))) return {Status::ResourceLimit};
  // General fixed-ready is deliberately unqualified. Zero means unavailable,
  // not a zero-cost callable stage; BuildFixedMain rejects that policy.
  const auto ready_scratch=policy==TopologyPolicy::ManifoldTwoSided?scratch.bytes():0;
  next.forecast={Status::Ok,persistent.bytes(),scratch.bytes(),ready.bytes(),ready_scratch,g,sides_only?0:4*g,sides_only?0:4*g};
  // Output's exact same prefix layout is used for private staging. The dead
  // sorted-edge region is first reusable for mixed IDEL1 solid-support buckets,
  // with the existing candidate IDs holding one main's four deduplicated lists.
  // Tags then become reference labels. Finally the dead sorted-edge region is
  // placement-constructed as neighbor-normal scratch;
  // no edge pointer survives that phase transition and no extra allocation exists.
  static_assert(sizeof(Edge)>=sizeof(StoredNormal));
  if (persistent.bytes()>limits.max_output_bytes || ready.bytes()>limits.max_output_bytes ||
      scratch.bytes()>limits.max_scratch_bytes) next.forecast.status=Status::ResourceLimit;
  output=next; return {next.forecast.status};
}
Data Construct(tl::util::HostArena& arena,const OutputLayout& p) noexcept {
  return {arena.Construct<Main>(p.mains),arena.Construct<std::uint32_t>(p.expanded_to_primary),
      arena.Construct<std::uint32_t>(p.primary_to_partner),
      p.normals.count ? arena.Construct<StoredNormal>(p.normals) : nullptr,
      p.references.count ? arena.Construct<NormalReference>(p.references) : nullptr,
      p.normal_offsets.count ? arena.Construct<std::uint32_t>(p.normal_offsets) : nullptr,
      p.normal_mains.count ? arena.Construct<std::uint32_t>(p.normal_mains) : nullptr,
      p.primary_roles.count ? arena.Construct<ShellSideRole>(p.primary_roles) : nullptr,
      p.primary_identities.count ? arena.Construct<PrimaryFaceIdentity>(p.primary_identities) : nullptr,
      p.mains.count,
      p.raw_origins.count ? arena.Construct<PrimaryFaceIdentity>(p.raw_origins) : nullptr,
      p.raw_origin_to_primary.count ? arena.Construct<std::uint32_t>(p.raw_origin_to_primary) : nullptr,
      p.post_gapm.count ? arena.Construct<PostGapmTopology>(p.post_gapm) : nullptr,
      p.post_corners.count ? arena.Construct<PrimaryCornerPermutation>(p.post_corners) : nullptr,
      p.post_before.count ? arena.Construct<PreShellSolidSupport>(p.post_before) : nullptr,
      p.post_support.count ? arena.Construct<PostGapmMainSupport>(p.post_support) : nullptr};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
namespace tlfea::contact::radioss_type25::startup {
Forecast Preflight(std::size_t nodes,std::size_t primary,Limits limits) noexcept {
  detail::Layout layout; const auto report=detail::MakeLayout(nodes,primary,limits,layout);
  if (layout.forecast.output_bytes) return layout.forecast;
  Forecast result; result.status=report.status; return result;
}
Forecast Preflight(const Input& input,Limits limits) noexcept {
  if(!role_policy::Supported(input.profile,input.topology) || role_policy::Mixed(input.topology)) {
    Forecast result;result.status=Status::UnsupportedProfile;return result;
  }
  detail::Layout layout;
  const auto report=detail::MakeLayout(input.node_count,input.primary_count,limits,layout,input.topology,input.shell_primary_count,input.raw_origin_count);
  if(layout.forecast.output_bytes)return layout.forecast;
  Forecast result;result.status=report.status;return result;
}
} // namespace tlfea::contact::radioss_type25::startup
