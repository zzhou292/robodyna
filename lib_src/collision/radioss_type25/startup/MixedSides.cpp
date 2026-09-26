// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25FixedMainStartup.h"
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup {
Forecast PreflightMixedSides(const Input& in,Limits limits) noexcept {
  if(in.profile!=Profile::MixedSurface || in.topology!=TopologyPolicy::NativeMixedSurface) {
    Forecast result;result.status=Status::UnsupportedProfile;return result;
  }
  detail::Layout layout;
  const auto report=detail::MakeLayout(in.node_count,in.primary_count,limits,layout,in.topology,
      in.shell_primary_count,in.raw_origin_count,true);
  if(layout.forecast.output_bytes)return layout.forecast;
  Forecast result;result.status=report.status;return result;
}
Report BuildMixedSides(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,MixedSidesSnapshot* published) noexcept {
  if(in.profile!=Profile::MixedSurface || in.topology!=TopologyPolicy::NativeMixedSurface)
    return {Status::UnsupportedProfile};
  detail::Layout layout;
  auto report=detail::MakeLayout(in.node_count,in.primary_count,limits,layout,in.topology,
      in.shell_primary_count,in.raw_origin_count,true);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.output_bytes || scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  report=detail::CheckInput(in,layout,output,scratch,published,sizeof(*published));
  if(report.status!=Status::Ok)return report;
  auto staged=detail::Construct(scratch,layout.output);
  report=detail::Expand(in,staged,scratch.Construct<Vector>(layout.points),
      scratch.Construct<detail::Identity>(layout.identities),scratch.Construct<detail::FaceKey>(layout.face_keys));
  if(report.status!=Status::Ok)return report;
  auto committed=detail::Construct(output,layout.output);
  const auto g=layout.forecast.expanded_mains,p=in.primary_count,n=in.raw_origin_count;
  std::copy_n(staged.mains,g,committed.mains);
  std::copy_n(staged.expanded_to_primary,g,committed.expanded_to_primary);
  std::copy_n(staged.primary_to_partner,p,committed.primary_to_partner);
  std::copy_n(staged.primary_roles,p,committed.primary_roles);
  std::copy_n(staged.primary_identities,p,committed.primary_identities);
  std::copy_n(staged.raw_origins,n,committed.raw_origins);
  std::copy_n(staged.raw_origin_to_primary,n,committed.raw_origin_to_primary);
  MixedSidesSnapshot next;
  next.mains=committed.mains;next.node_count=in.node_count;next.primary_count=p;
  next.main_count=g;next.shell_primary_count=in.shell_primary_count;
  next.expanded_to_primary=committed.expanded_to_primary;
  next.primary_to_partner=committed.primary_to_partner;
  next.primary_roles=committed.primary_roles;next.primary_identities=committed.primary_identities;
  next.raw_origins=committed.raw_origins;next.raw_origin_to_primary=committed.raw_origin_to_primary;
  next.raw_origin_count=n;next.source_generation=in.source_generation;
  *published=next;return {Status::Ok};
}
}
