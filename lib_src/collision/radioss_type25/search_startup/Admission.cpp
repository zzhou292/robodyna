// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../search/Ranges.h"
#include "../../self_contact_filters/Environment.h"
#include "lib_src/math/ScalarBits.h"
namespace tlfea::contact::radioss_type25::search_startup::detail {
namespace {
bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  if (!an || !bn) return true;
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a && b && an<=UINTPTR_MAX-x && bn<=UINTPTR_MAX-y && (x+an<=y || y+bn<=x);
}
}
Report Admit(const Input& in,const Layout&,Limits,const tl::util::HostArena& output,
    const tl::util::HostArena& scratch,const void* result,std::size_t result_bytes) noexcept {
  namespace r=search::detail;
  const auto& p=in.profile;const auto& c=in.contributors;const auto& mesh=in.mesh;const auto& top=in.topology;
  if (p.level!=1 || p.gap_mode!=1 || p.neighbor_removal!=2 || p.initial_penetration!=5 ||
      p.edge_mode!=0 || p.thermal_mode!=0 || (p.curvature!=0 && p.curvature!=1) || p.partitions!=1 ||
      p.gap_load_cards!=LoadCards::Absent ||
      (p.initialization!=Initialization::SerialNative && p.initialization!=Initialization::InvariantNoExpansion) ||
      c.census!=Census::CompleteDeclaredModel || c.physical_nodes!=mesh.node_count ||
      c.physical_shells<mesh.primary_count || c.other_interfaces || c.unsupported_elements ||
      (mesh.profile!=startup::Profile::OrdinaryExteriorFixedMain &&
       mesh.profile!=startup::Profile::OrdinaryExteriorMovingMain) ||
      (mesh.topology!=startup::TopologyPolicy::ManifoldTwoSided &&
       mesh.topology!=startup::TopologyPolicy::NativeOrdinaryShell))
    return {Status::UnsupportedProfile};
  if (!self_contact_filters::CompatibleHostArithmetic()) return {Status::UnsupportedArithmetic};
  const auto n=mesh.node_count,g=2*mesh.primary_count,s=in.secondary_count;
  std::size_t positions_bytes=0;
  if (top.profile!=mesh.profile || top.topology!=mesh.topology || !mesh.source_generation || top.source_generation!=mesh.source_generation || top.node_count!=n ||
      top.primary_count!=mesh.primary_count || top.main_count!=g || in.main_count!=g ||
      !r::Span(mesh.node_source_ids,n) || !r::Span(mesh.primary,mesh.primary_count) ||
      !r::VectorSpan(mesh.positions,n,positions_bytes) || !r::Span(top.mains,g) ||
      !r::Span(in.secondary,s) || !r::Span(in.main_gaps,g) || !r::Span(static_cast<const std::byte*>(result),result_bytes) ||
      !r::Span(in.auxiliary_rigid_primary_ids,in.auxiliary_rigid_primary_count)) return {Status::InvalidInput};
  if (mesh.coordinates!=startup::Coordinates::Native && mesh.coordinates!=startup::Coordinates::Si)
    return {Status::InvalidInput};
  if (mesh.coordinates==startup::Coordinates::Si) {
    units_detail::Factors factors;
    if (!units_detail::Make(mesh.units,factors)) return {Status::InvalidInput};
  }
  struct Range {const void* pointer;std::size_t bytes;};
  const Range reads[]{{&in,sizeof(in)},{&output,sizeof(output)},{&scratch,sizeof(scratch)},
      {in.auxiliary_rigid_primary_ids,in.auxiliary_rigid_primary_count*sizeof(std::uint64_t)},{mesh.node_source_ids,n*sizeof(std::uint64_t)},
      {mesh.primary,mesh.primary_count*sizeof(startup::PrimaryFace)},{mesh.positions.data,positions_bytes},
      {top.mains,g*sizeof(startup::Main)},{in.secondary,s*sizeof(Secondary)},{in.main_gaps,g*sizeof(double)}};
  if (!Disjoint(output.data(),output.bytes(),scratch.data(),scratch.bytes()) ||
      !Disjoint(output.data(),output.bytes(),result,result_bytes) ||
      !Disjoint(scratch.data(),scratch.bytes(),result,result_bytes))return {Status::InvalidInput};
  for (const auto& read:reads)
    if (!Disjoint(output.data(),output.bytes(),read.pointer,read.bytes) ||
        !Disjoint(scratch.data(),scratch.bytes(),read.pointer,read.bytes) ||
        !Disjoint(result,result_bytes,read.pointer,read.bytes))return {Status::InvalidInput};
  constexpr unsigned reverse[]{1,0,3,2};
  for (std::size_t m=0;m<g;++m) {
    const auto primary=m<mesh.primary_count?m:m-mesh.primary_count;
    const auto& main=top.mains[m];const auto& original=mesh.primary[primary];
    if (main.source_id!=original.source_id || main.global_id!=static_cast<int>(m+1) ||
        main.segment_type!=(m<mesh.primary_count?static_cast<int>(mesh.primary_count+m+1):-static_cast<int>(primary+1)))
      return {Status::InvalidInput,m};
    for (unsigned k=0;k<4;++k)
      if (main.nodes[k]>=n || main.nodes[k]!=original.nodes[m<mesh.primary_count?k:reverse[k]])
        return {Status::InvalidInput,m};
    if (!std::isfinite(in.main_gaps[m]) || in.main_gaps[m]<0) return {Status::InvalidInput,m};
    if (m>=mesh.primary_count && !tl::math::SameScalarBits(in.main_gaps[m],in.main_gaps[primary]))
      return {Status::InvalidInput,m};
  }
  return {Status::Ok};
}
Report Prepare(const Input& in,Work work,double& maximum_secondary_gap,double& total_gap) noexcept {
  const auto n=in.mesh.node_count;
  const double length=in.mesh.coordinates==startup::Coordinates::Native?1.:in.mesh.units.length_m;
  for (std::size_t i=0;i<n;++i) {
    const auto value=in.mesh.positions.at(static_cast<std::uint32_t>(i));
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
      return {Status::InvalidInput,SIZE_MAX,i};
    const Vector converted{value.x/length,value.y/length,value.z/length};
    if (!std::isfinite(converted.x) || !std::isfinite(converted.y) || !std::isfinite(converted.z))
      return {Status::NonfiniteResult,SIZE_MAX,i};
    work.points[i]=converted;work.secondary_index[i]=UINT32_MAX;work.secondary_gap[i]=0;
  }
  maximum_secondary_gap=0;
  for (std::size_t i=0;i<in.secondary_count;++i) {
    const auto row=in.secondary[i];
    if (row.node>=n || !std::isfinite(row.stiffness) || row.stiffness<0 ||
        !std::isfinite(row.gap) || row.gap<0 || work.secondary_index[row.node]!=UINT32_MAX)
      return {Status::InvalidInput,SIZE_MAX,row.node};
    work.secondary_index[row.node]=static_cast<std::uint32_t>(i);work.secondary_gap[row.node]=row.gap;
    maximum_secondary_gap=std::max(maximum_secondary_gap,row.gap);
  }
  double main_maximum=0;
  for (std::size_t i=0;i<in.mesh.primary_count;++i)main_maximum=std::max(main_maximum,in.main_gaps[i]);
  // I25STI3 final rawIGAP1 GAP assignment, from actual resolved row fields.
  total_gap=maximum_secondary_gap+main_maximum;
  if (!std::isfinite(total_gap)) return {Status::NonfiniteResult};
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::search_startup::detail
