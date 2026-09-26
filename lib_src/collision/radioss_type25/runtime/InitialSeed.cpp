// SPDX-License-Identifier: AGPL-3.0-or-later
#include "InitialSeed.h"
#include "../initial_source/Internal.h"
#include "lib_src/math/ScalarBits.h"
#include <cstddef>
#include <type_traits>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
namespace is=initial_source;
namespace ld=lifecycle::detail;
TransactionReport Fail(TransactionStatus status,const char* message,std::size_t row=SIZE_MAX){return {status,message,row};}
bool Same(double a,double b){return tl::math::SameScalarBits(a,b);}
bool Same(StoredNormal a,StoredNormal b){return tl::math::SameScalarBits(a.x,b.x)&&tl::math::SameScalarBits(a.y,b.y)&&tl::math::SameScalarBits(a.z,b.z);}
bool Same(const lifecycle::NormalReference& a,const lifecycle::NormalReference& b){return a.boundary==b.boundary&&Same(a.bisector[0],b.bisector[0])&&Same(a.bisector[1],b.bisector[1]);}
bool Same(const startup::PrimaryFaceIdentity& a,const startup::PrimaryFaceIdentity& b) {
  return a.kind==b.kind&&a.physical_parent_id==b.physical_parent_id&&a.local_face==b.local_face&&a.origin==b.origin&&a.origin_count==b.origin_count;
}
bool Same(const startup::PostGapmMainSupport& a,const startup::PostGapmMainSupport& b) {
  return a.first.kind==b.first.kind&&a.first.source_element_id==b.first.source_element_id&&a.second_solid_source_id==b.second_solid_source_id;
}
bool Add(std::size_t value,std::size_t& target){if(value>SIZE_MAX-target)return false;target+=value;return true;}
bool Topology(const startup::Snapshot& a,const is::detail::Prepared& p) {
  const auto& b=p.starter;const auto g=p.mains.size(),primary=b.primary_count,r=p.references.size();
  if(a.profile!=b.profile||a.topology!=b.topology||a.node_count!=b.node_count||a.primary_count!=primary||a.main_count!=g||
      a.source_generation!=b.source_generation||a.starter.reference_count!=r||a.normal_incidence_count!=p.normal_mains.size()||
      a.primary_role_count!=p.primary_roles.size()||a.primary_identity_count!=p.primary_identities.size()||
      a.shell_primary_count!=b.shell_primary_count||a.raw_origin_count!=p.raw_origins.size()||bool(a.post_gapm)!=bool(b.post_gapm)||
      !ld::Span(a.mains,g)||!ld::Span(a.expanded_to_primary,g)||!ld::Span(a.primary_to_partner,primary)||
      !ld::Span(a.normal_offsets,r+1)||!ld::Span(a.normal_mains,a.normal_incidence_count)||
      !ld::Span(a.primary_roles,a.primary_role_count)||!ld::Span(a.primary_identities,a.primary_identity_count)||
      !ld::Span(a.raw_origins,a.raw_origin_count)||!ld::Span(a.raw_origin_to_primary,a.raw_origin_count)||
      !ld::Span(a.starter.face_normals,4*g)||!ld::Span(a.starter.references,r))return false;
  for(std::size_t i=0;i<g;++i) {
    const auto& x=a.mains[i];const auto& y=p.topology[i];
    if(x.source_id!=y.source_id||x.global_id!=y.global_id||x.segment_type!=y.segment_type||a.expanded_to_primary[i]!=p.expanded_to_primary[i])return false;
    for(unsigned k=0;k<4;++k)if(x.nodes[k]!=y.nodes[k]||x.neighbors[k]!=y.neighbors[k]||x.neighbor_edges[k]!=y.neighbor_edges[k]||
        x.normal_reference[k]!=y.normal_reference[k]||!Same(a.starter.face_normals[4*i+k],p.mains[i].normal_slot[k]))return false;
  }
  for(std::size_t i=0;i<primary;++i)if(a.primary_to_partner[i]!=p.primary_to_partner[i])return false;
  for(std::size_t i=0;i<r;++i)if(!Same(a.starter.references[i],p.references[i]))return false;
  for(std::size_t i=0;i<=r;++i)if(a.normal_offsets[i]!=p.normal_offsets[i])return false;
  for(std::size_t i=0;i<p.normal_mains.size();++i)if(a.normal_mains[i]!=p.normal_mains[i])return false;
  for(std::size_t i=0;i<p.primary_roles.size();++i)if(a.primary_roles[i]!=p.primary_roles[i])return false;
  for(std::size_t i=0;i<p.primary_identities.size();++i)if(!Same(a.primary_identities[i],p.primary_identities[i]))return false;
  for(std::size_t i=0;i<p.raw_origins.size();++i)
    if(!Same(a.raw_origins[i],p.raw_origins[i])||a.raw_origin_to_primary[i]!=p.raw_origin_to_primary[i])return false;
  if(a.post_gapm) {
    if(!ld::Span(a.post_gapm,1))return false;const auto& x=*a.post_gapm;const auto& y=p.post_gapm;
    if(x.phase!=y.phase||x.primary_count!=primary||x.before_shell_count!=primary||x.main_count!=g||
        x.pre_shell_internal_count!=y.pre_shell_internal_count||x.incoming_solid_erosion!=y.incoming_solid_erosion||
        x.final_solid_erosion!=y.final_solid_erosion||x.source_generation!=y.source_generation||
        !ld::Span(x.primary_corners,primary)||!ld::Span(x.before_shell,primary)||!ld::Span(x.final_support,g))return false;
    for(std::size_t i=0;i<primary;++i) {
      for(unsigned k=0;k<4;++k)if(x.primary_corners[i].source_corner[k]!=p.primary_corners[i].source_corner[k])return false;
      const auto& one=x.before_shell[i];const auto& two=p.before_shell[i];
      if(one.first_solid_source_id!=two.first_solid_source_id||one.second_solid_source_id!=two.second_solid_source_id||one.unique_match_count!=two.unique_match_count)return false;
    }
    for(std::size_t i=0;i<g;++i)if(!Same(x.final_support[i],p.final_support[i]))return false;
  }
  return true;
}
}
InitialMainRoster InitialSeedAccess::MainRoster(const is::PreparedSource& prepared) noexcept {
  if(!prepared.impl_)return {};
  const auto& nodes=prepared.impl_->source.main_nodes;return {nodes.data(),nodes.size()};
}
TransactionReport InitialSeedAccess::Bind(const TransactionConfig& config,const ContactSourceInput& source,
    const is::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    const startup::Snapshot* starter,const startup::FixedMainView* ready,lifecycle::SourceView& output) noexcept {
  if(!prepared.impl_)return Fail(TransactionStatus::SourceMismatch,"General contact requires genuine prepared initial source");
  const auto& owned=*prepared.impl_;const auto& p=owned.source;const auto& id=owned.identity;
  const auto& in=p.descriptor;const auto& c=in.controls;const auto& s=source.selection;
  if(id.input_phase!=is::Phase::StarterNormalsAndPreBucGaps||id.engine_handoff!=is::EngineHandoff::SourceProvedFreshSerialSearchAtZero)
    return Fail(TransactionStatus::UnsupportedProfile,"General contact requires source-proved serial fresh search at time zero");
  const auto* domain=physical.domain();
  if(!physical.prepared()||!domain||!domain->prepared()||id.source.physical_domain!=domain->source_instance_id()||
      !id.source.runtime_topology||source.topology_generation!=id.source.runtime_topology||source.source_id!=id.source.source||
      s.generation!=id.source.topology||source.primary_main_count!=id.primaries||s.node_count!=id.nodes||
      s.secondary_count!=id.secondaries||s.main_count!=id.mains||s.normal_count!=id.references||domain->node_count()!=id.nodes||
      !Same(config.units.length_m,id.units.length_m)||!Same(config.units.mass_kg,id.units.mass_kg)||!Same(config.units.time_s,id.units.time_s))
    return Fail(TransactionStatus::SourceMismatch,"General source identity/count/domain/unit mismatch");
  const auto& geometry=config.lifecycle.geometry;const auto& selection=config.lifecycle.selection;
  if(geometry.gap_mode!=c.gap_mode||geometry.initial_penetration!=c.initial_penetration||geometry.damping_flag!=c.damping_flag||geometry.sharp!=c.sharp||
      selection.gap_mode!=c.gap_mode||selection.initial_penetration!=c.initial_penetration||selection.local_processor!=1||
      config.lifecycle.neighbor_removal!=c.neighbor_removal||config.lifecycle.optcd_response_precision!=c.arithmetic_precision||
      config.lifecycle.coefficient.stiffness_formulation!=c.stiffness_formulation||
      config.lifecycle.coefficient.mass_timestep_augmentation!=c.stiffness_mass_update||source.native_workers!=c.starter_workers||
      source.force_packet_size!=unsigned(c.native_packet_size)||source.contact_thickness_update!=c.thickness_update||
      !Same(source.margin,p.diagnostics.engine_margin)||!Same(source.drad,c.drad)||!Same(source.gap_load,c.gap_load))
    return Fail(TransactionStatus::SourceMismatch,"General runtime and initial native controls differ");
  const auto count=id.nodes,g=id.mains,r=id.references,rows=id.secondaries;
  if(!ld::Span(s.nodes,count)||!ld::Span(s.mains,g)||!ld::Span(s.secondary,rows)||!ld::Span(s.normals,r)||
      !ld::Span(source.primary_curvature,id.primaries)||s.normal_to_main.offset_count!=p.normal_offsets.size()||
      s.normal_to_main.entry_count!=p.normal_mains.size()||!ld::Span(s.normal_to_main.offsets,p.normal_offsets.size())||
      !ld::Span(s.normal_to_main.entries,p.normal_mains.size()))return Fail(TransactionStatus::InvalidInput,"General source spans differ");
  if(bool(starter)==bool(ready))return Fail(TransactionStatus::SourceMismatch,"General source normal phase is ambiguous");
  if(starter&&!Topology(*starter,p))return Fail(TransactionStatus::SourceMismatch,"General Starter topology/maps/provenance differ");
  if(ready&&(p.starter.profile!=startup::Profile::OrdinaryExteriorFixedMain||ready->profile!=p.starter.profile||
      ready->topology!=p.starter.topology||ready->source_generation!=id.source.topology||ready->normals.reference_count!=r||
      !ld::Span(ready->normals.face_normals,4*g)||!ld::Span(ready->normals.references,r)))
    return Fail(TransactionStatus::SourceMismatch,"General fixed source requires its distinct genuine ready-normal phase");
  if(ready) {
    if(!ld::Span(source.primary_parent_ids,id.primaries))return Fail(TransactionStatus::InvalidInput,"General fixed primary identity is unavailable");
    for(std::size_t i=0;i<id.primaries;++i)if(source.primary_parent_ids[i]!=p.topology[i].source_id)
      return Fail(TransactionStatus::SourceMismatch,"General fixed physical parent identity differs",i);
  }
  for(std::size_t i=0;i<count;++i) {
    if(s.nodes[i].source_id!=p.nodes[i].source_id||s.nodes[i].constraint!=p.nodes[i].constraint||s.nodes[i].skew!=p.nodes[i].skew||
        domain->nodes()[i].source_id!=p.nodes[i].source_id)return Fail(TransactionStatus::SourceMismatch,"General physical node map differs",i);
    const auto expected=domain->nodes()[i].position;const auto x=p.positions[i];
    const double native[]{x.x,x.y,x.z},si[]{expected.x,expected.y,expected.z};
    for(unsigned k=0;k<3;++k) {
      const double converted=native[k]*id.units.length_m;
      if(!tl::math::Finite(converted)||(native[k]!=0&&converted==0)||!Same(converted,si[k]))
        return Fail(TransactionStatus::SourceMismatch,"General native working coordinates differ from actual SI domain",i);
    }
  }
  for(std::size_t m=0;m<g;++m) {
    const auto& a=s.mains[m];const auto& b=p.mains[m];
    if(a.global_id!=b.global_id||a.segment_type!=b.segment_type||!Same(a.coefficient,b.coefficient)||!Same(a.maximum_gap,b.maximum_gap))
      return Fail(TransactionStatus::SourceMismatch,"General main identity/coefficient/search gap differs",m);
    for(unsigned k=0;k<4;++k)if(a.nodes[k]!=b.nodes[k]||a.neighbors[k]!=b.neighbors[k]||a.normal_reference[k]!=b.normal_reference[k]||
        !Same(a.gap[k],b.gap[k])||!Same(a.normal_slot[k],ready?ready->normals.face_normals[4*m+k]:b.normal_slot[k]))
      return Fail(TransactionStatus::SourceMismatch,"General main topology/corner/normal source phase differs",m);
  }
  for(std::size_t row=0;row<rows;++row) {
    const auto& a=s.secondary[row];const auto& b=p.secondary[row];
    if(a.node!=b.node||!Same(a.coefficient,b.coefficient)||!Same(a.gap,b.gap)||a.initial_contact_flag!=b.initial_contact_flag)
      return Fail(TransactionStatus::SourceMismatch,"General NSV/coefficient/gap order differs",row);
  }
  for(std::size_t i=0;i<r;++i)if(!Same(s.normals[i],ready?ready->normals.references[i]:p.references[i]))
    return Fail(TransactionStatus::SourceMismatch,"General normal-reference phase differs",i);
  for(std::size_t i=0;i<p.normal_offsets.size();++i)if(s.normal_to_main.offsets[i]!=p.normal_offsets[i])return Fail(TransactionStatus::SourceMismatch,"General normal offsets differ",i);
  for(std::size_t i=0;i<p.normal_mains.size();++i)if(s.normal_to_main.entries[i]!=p.normal_mains[i])return Fail(TransactionStatus::SourceMismatch,"General normal incidence differs",i);
  for(std::size_t i=0;i<id.primaries;++i)if(!Same(source.primary_curvature[i],p.primary_extent[i]))return Fail(TransactionStatus::SourceMismatch,"General source-produced primary extent differs",i);
  const auto& old=s.removed_main_by_secondary;
  if(old.offsets||old.offset_count||old.entries||old.entry_count) {
    if(old.offset_count!=p.final_offsets.size()||old.entry_count!=p.final_mains.size()||!ld::Span(old.offsets,old.offset_count)||!ld::Span(old.entries,old.entry_count))
      return Fail(TransactionStatus::SourceMismatch,"General supplied removal descriptor differs from source producer");
    for(std::size_t i=0;i<p.final_offsets.size();++i)if(old.offsets[i]!=p.final_offsets[i])return Fail(TransactionStatus::SourceMismatch,"General removal offsets differ",i);
    for(std::size_t i=0;i<p.final_mains.size();++i)if(old.entries[i]!=p.final_mains[i])return Fail(TransactionStatus::SourceMismatch,"General removal order differs",i);
  }
  auto next=s;next.removed_main_by_secondary=p.descriptor.contact.removed_main_by_secondary;output=next;
  return {TransactionStatus::Ok,"OK"};
}
TransactionReport InitialSeedAccess::Forecast(const is::PreparedSource& source,const TransactionForecast& runtime,
    TransactionLimits limits,GeneralTransactionForecast& output) noexcept {
  if(!source.impl_)return Fail(TransactionStatus::SourceMismatch,"General initial source is unavailable");
  GeneralTransactionForecast next;next.transaction=runtime;next.initializer=source.impl_->forecast;
  next.peak_device_bytes=runtime.device_bytes;next.peak_host_bytes=runtime.startup_host_bytes;
  if(!Add(next.initializer.peak_device_bytes,next.peak_device_bytes)||next.peak_device_bytes>limits.max_device_bytes||
      !Add(next.initializer.retained_host_bytes,next.peak_host_bytes)||next.peak_host_bytes>limits.max_host_bytes)
    return Fail(TransactionStatus::ResourceLimit,"General runtime/producer coexistence exceeds cap");
  output=next;return {TransactionStatus::Ok,"OK"};
}
TransactionReport InitialSeedAccess::Upload(const is::PreparedSource& source,Device device,cudaStream_t stream,
    TransactionInitializationDiagnostics& output) noexcept {
  is::DeviceSeed seed;const auto report=is::Prepare(source,stream,seed);
  if(report.status!=is::Status::Ok)return Fail(report.status==is::Status::ResourceLimit?TransactionStatus::ResourceLimit:
      report.status==is::Status::DeviceFailure?TransactionStatus::DeviceFailure:TransactionStatus::NumericalFailure,
      "Genuine initial GPU history production failed",report.row);
  if(report.diagnostics.warm_negative_main)return Fail(TransactionStatus::UnsupportedProfile,"Native negative warm-main continuation is not yet qualified");
  const auto& p=*seed.impl_;static_assert(std::is_standard_layout_v<lifecycle::Main>&&std::is_standard_layout_v<lifecycle::Secondary>);
  const auto* history=tl::util::ArenaPointer<NativeGeometryHistory>(p.data,p.layout.history);
  const auto* flags=tl::util::ArenaPointer<int>(p.data,p.layout.flags);
  const auto* corners=tl::util::ArenaPointer<double>(p.data,p.layout.corner_gaps);
  cudaError_t error=cudaSuccess;
  for(unsigned slab=0;slab<2;++slab) {
    if(error==cudaSuccess)error=cudaMemcpyAsync(device.history[slab],history,p.layout.history.bytes,cudaMemcpyDeviceToDevice,stream);
    auto* target=reinterpret_cast<std::byte*>(device.secondary[slab])+offsetof(lifecycle::Secondary,initial_contact_flag);
    if(error==cudaSuccess)error=cudaMemcpy2DAsync(target,sizeof(lifecycle::Secondary),flags,sizeof(int),sizeof(int),p.identity.secondaries,cudaMemcpyDeviceToDevice,stream);
  }
  auto* target=reinterpret_cast<std::byte*>(const_cast<lifecycle::Main*>(device.source.mains))+offsetof(lifecycle::Main,gap);
  if(error==cudaSuccess)error=cudaMemcpy2DAsync(target,sizeof(lifecycle::Main),corners,4*sizeof(double),4*sizeof(double),p.identity.mains,cudaMemcpyDeviceToDevice,stream);
  const auto drained=cudaStreamSynchronize(stream); // Private seed outlives every D2D copy, including failure paths.
  if(error!=cudaSuccess||drained!=cudaSuccess)return Fail(TransactionStatus::DeviceFailure,"General initial history handoff failed");
  output={true,seed.identity(),report.diagnostics};
  return {TransactionStatus::Ok,"OK"};
}
}
