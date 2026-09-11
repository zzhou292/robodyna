// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchStorage.h"
#include "Type25Stability.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"
#include <limits>

namespace tl::fea::type25::batch_detail {
BatchReport BuildStartup(const BatchConfig& c,const Model& source,const NodalMassBinding& mass,
    util::HostArena& arena,const ArenaLayout& layout,Storage& header,BatchDiagnostics& diagnostics) {
  const auto& a=c.owner;
  if(!a.owner_id||a.epoch||a.time!=0||a.velocity_time!=0||!a.has_rotations||
     a.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||
     a.velocity_phase!=NodalVelocityPhase::Collocated||!detail::Positive(a.fixed_dt)||
     a.reactions_valid||a.reaction_base_epoch||a.reaction_time!=0||a.reaction_kick_dt!=0||
     !c.configuration_id||!c.qualification_id||!shell_startup_detail::ValidStartup(c.startup,true)||
     !native_physical_coefficients::ValidScope(a.rigid_groups,a.node_count))
    return {BatchStatus::InvalidInput,"TYPE25 requires a fresh extended staggered owner and explicit startup"};
  if(!source.prepared()||!mass.prepared()||!mass.Matches(source)||
     source.connection_count()!=c.element_count||source.global_node_count()!=a.node_count||
     mass.node_count()!=a.node_count||mass.source_instance_id()!=source.source_instance_id()||
     (!native_physical_coefficients::Empty(a.rigid_groups)&&a.rigid_groups.source_instance_id!=source.source_instance_id()))
    return {BatchStatus::InvalidInput,"TYPE25 model/combined mass/complete owner source identities differ"};
  Storage next;
  auto constructed=ConstructStartup(c,source,arena,layout,next);
  if(constructed.status!=BatchStatus::Success)return constructed;
  for(std::size_t n=0;n<a.node_count;++n) {
    const auto& node=mass.nodes()[n];const auto& coefficients=node.coefficients;
    if(!detail::Positive(coefficients.mass)||!detail::Positive(coefficients.isotropic_inertia))
      return {BatchStatus::InvalidMass,"TYPE25 combined nodal coefficients must be finite and positive",UINT32_MAX,static_cast<std::uint32_t>(n)};
    next.model.nodes[n]={node.position,coefficients.mass,coefficients.isotropic_inertia};
  }
  auto built=BuildElements(c,source,next,diagnostics);
  if(built.status!=BatchStatus::Success)return built;
  next.control.diagnostics=diagnostics;
  *util::ArenaPointer<Storage>(arena.data(),layout.header)=next;
  header=next;
  return {BatchStatus::Success,"OK"};
}
BatchReport ConstructStartup(const BatchConfig& c,const Model& source,util::HostArena& arena,
    const ArenaLayout& layout,Storage& header) {
  auto* initial=arena.Construct<Storage>(layout.header);
  if(!initial||!arena.Construct<Property>(layout.properties)||!arena.Construct<DeviceElement>(layout.elements)||
     !arena.Construct<DeviceNode>(layout.nodes)||!arena.Construct<Evaluation>(layout.slab[0])||
     !arena.Construct<Evaluation>(layout.slab[1])||!arena.Construct<Status>(layout.status))
    return {BatchStatus::ResourceLimit,"TYPE25 startup arena cannot construct its admitted records"};
  auto next=RebasedHeader(arena.data(),layout);
  next.model.config=c;next.model.units=source.source_units();next.model.source_instance_id=source.source_instance_id();
  for(std::size_t p=0;p<source.property_count();++p)next.model.properties[p]=source.properties()[p].property;
  header=next;
  return {BatchStatus::Success,"OK"};
}
BatchReport BuildElements(const BatchConfig& c,const Model& source,Storage& header,
    BatchDiagnostics& diagnostics) {
  double minimum=std::numeric_limits<double>::max();
  for(std::size_t e=0;e<c.element_count;++e) {
    const auto& connection=source.connections()[e];const auto& reference=source.references()[e];
    header.model.elements[e]={reference,{connection.global_node[0],connection.global_node[1]},connection.property_index};
    Evaluation evaluation;evaluation.history=source.initial_histories()[e];
    const auto x=tl::math::fixed3::Divide(tl::math::fixed3::Subtract(reference.position[1],reference.position[0]),reference.length_m);
    const auto y=reference.transverse_axis,z=tl::math::fixed3::Cross(x,y);
    evaluation.frame.axes=evaluation.frame.midpoint_axes=tl::math::fixed3::Columns(x,y,z);
    evaluation.frame.length_m=evaluation.frame.midpoint_length_m=reference.length_m;
    Stability stability;const auto checked=CriticalStep(source.source_units(),header.model.properties[connection.property_index],reference.length_m,stability);
    if(checked!=Status::Success)return {BatchStatus::ElementFailure,"Invalid TYPE25 initial coefficient bound",static_cast<std::uint32_t>(e),UINT32_MAX,checked};
    evaluation.critical_dt_s=stability.critical_dt_s;
    evaluation.translation_stiffness_N_per_m=stability.translation_stiffness_N_per_m;
    evaluation.rotation_stiffness_Nm_per_rad=stability.rotation_stiffness_Nm_per_rad;
    header.slab[0].element[e]=evaluation;header.slab[1].element[e]=evaluation;
    if(stability.critical_dt_s<minimum)minimum=stability.critical_dt_s;
  }
  diagnostics=InitialDiagnostics(c,source.source_instance_id(),minimum);
  header.control.diagnostics=diagnostics;
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::type25::batch_detail
