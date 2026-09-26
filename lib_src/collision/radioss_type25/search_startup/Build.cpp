// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25SearchStartup.h"
#include "Internal.h"
#include "../search/Ranges.h"
namespace tlfea::contact::radioss_type25::search_startup {
namespace {
Report BuildValue(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,void* descriptor,std::size_t descriptor_bytes,
    detail::Context context,Snapshot& value) noexcept {
  std::size_t native_nodes=0;
  auto report=detail::ResolveContext(in,limits,context,native_nodes);
  if(report.status!=Status::Ok)return report;
  const bool composed=context==detail::Context::ComposedNoTied||context==detail::Context::ComposedBeforeTied;
  const bool mixed=composed&&in.mesh.profile==startup::Profile::MixedSurface;
  detail::Layout layout;
  report=detail::MakeLayout(in.mesh.node_count,in.mesh.primary_count,in.secondary_count,
      limits,layout,in.auxiliary_rigid_primary_count,mixed?in.main_count:0);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.output_bytes || scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  report=mixed?detail::AdmitMixed(in,layout,limits,output,scratch,descriptor,descriptor_bytes):
      detail::Admit(in,layout,limits,output,scratch,descriptor,descriptor_bytes,composed);
  if(report.status!=Status::Ok)return report;
  auto staged=detail::Construct(scratch,layout.output);
  auto work=detail::ConstructWork(scratch,layout);
  report=detail::CheckAuxiliaryIds(in,work);
  if(report.status!=Status::Ok)return report;
  double secondary_gap=0,gap=0;
  report=detail::Prepare(in,work,secondary_gap,gap);
  if(report.status!=Status::Ok)return report;
  if(in.global_gap_phase==GlobalGapPhase::ExplicitPreNodalUpdate) {
    if(!composed||!std::isfinite(in.source_global_gap)||in.source_global_gap<0)return {Status::UnsupportedProfile};
    gap=in.source_global_gap;
  } else if(in.global_gap_phase!=GlobalGapPhase::OrdinaryResolvedFields)return {Status::UnsupportedProfile};
  Snapshot next;
  auto status=ResolveMultiplier(native_nodes,&next.multiplier);
  if(status!=Status::Ok)return {status};
  report=detail::Margin(in,work.points,next.multiplier,gap,limits,next.mean_length,next.margin);
  if(report.status!=Status::Ok)return report;
  report=detail::Extent(in,work.points,staged.extent,next.maximum_extent);
  if(report.status!=Status::Ok)return report;
  report=detail::Removals(in,limits,work,staged,layout.forecast.removal_capacity,secondary_gap);
  if(report.status!=Status::Ok)return report;
  // Source allocation initializes ICONT_I=0; selected INACTI5 PWR3 never
  // writes it. This does not replace native initial penetration/history work.
  next.removal_count=report.required_removals;
  const auto committed=detail::Construct(output,layout.output);
  std::copy_n(staged.extent,in.mesh.primary_count,committed.extent);
  std::copy_n(staged.main_offsets,in.main_count+1,committed.main_offsets);
  std::copy_n(staged.secondary_offsets,in.secondary_count+1,committed.secondary_offsets);
  std::copy_n(staged.removed_nodes,next.removal_count,committed.removed_nodes);
  std::copy_n(staged.removed_mains,next.removal_count,committed.removed_mains);
  std::copy_n(staged.contact,in.secondary_count,committed.contact);
  next.primary_extent=committed.extent;next.primary_count=in.mesh.primary_count;
  next.main_offsets=committed.main_offsets;next.secondary_offsets=committed.secondary_offsets;
  next.removed_nodes=next.removal_count?committed.removed_nodes:nullptr;
  next.removed_mains=next.removal_count?committed.removed_mains:nullptr;
  next.initial_contact=committed.contact;
  next.main_count=in.main_count;next.secondary_count=in.secondary_count;
  next.source_generation=in.mesh.source_generation;
  next.native_population=detail::ResolvedPopulation(in,native_nodes);
  next.native_model_nodes_exact=in.native_population.policy==NativePopulationPolicy::ExactDeclaredAuxiliaryIds;
  next.native_model_nodes=next.native_model_nodes_exact?native_nodes:0;
  value=next;return report;
}
}
Report Build(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  if(!search::detail::Span(published,1))return {Status::InvalidInput};
  Snapshot next;
  const auto report=BuildValue(in,limits,output,scratch,published,sizeof(*published),
      detail::Context::LegacyNoKinematics,next);
  if(report.status==Status::Ok)*published=next;
  return report;
}
Report BuildRigidOnly(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  if(!search::detail::Span(published,1))return {Status::InvalidInput};
  Snapshot next;
  const auto report=BuildValue(in,limits,output,scratch,published,sizeof(*published),
      detail::Context::RigidOnly,next);
  if(report.status==Status::Ok)*published=next;
  return report;
}
Report BuildGeometricBeforeTied(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,GeometricSnapshot* published) noexcept {
  if(!search::detail::Span(published,1))return {Status::InvalidInput};
  Snapshot next;
  const auto report=BuildValue(in,limits,output,scratch,published,sizeof(*published),
      detail::Context::BeforeTied,next);
  if(report.status==Status::Ok)*published={next,in.contributors};
  return report;
}
Report BuildComposedNoTied(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  if(!search::detail::Span(published,1))return {Status::InvalidInput};
  Snapshot next;const auto report=BuildValue(in,limits,output,scratch,published,sizeof(*published),
      detail::Context::ComposedNoTied,next);
  if(report.status==Status::Ok)*published=next;
  return report;
}
Report BuildComposedGeometricBeforeTied(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,GeometricSnapshot* published) noexcept {
  if(!search::detail::Span(published,1))return {Status::InvalidInput};
  Snapshot next;const auto report=BuildValue(in,limits,output,scratch,published,sizeof(*published),
      detail::Context::ComposedBeforeTied,next);
  if(report.status==Status::Ok)*published={next,in.contributors,in.covered_type25_siblings};
  return report;
}
} // namespace tlfea::contact::radioss_type25::search_startup
