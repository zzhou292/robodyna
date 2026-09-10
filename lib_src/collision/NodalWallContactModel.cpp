#include "NodalWallContactState.h"
#include <cmath>
#include <new>

namespace tlfea::contact::nodal_wall_device_detail {
NodalWallDeviceReport PrepareModel(const NodalWallDeviceConfig& c,PlanarWallView source,
    const NodalWallWeights& weights,VectorView x,const double* inverse,const std::uint8_t* masks,
    PlanarWallBox motion,PreparedModel* output) {
  using fea=tl::fea::NodalTemporalScheme;
  const auto& owner=c.owner; const auto& law=c.law;
  if (!output || !x.valid() || !inverse || !masks || !weights.prepared() || !owner.owner_id ||
      !c.configuration_id || !c.qualification_id || !c.wall_binding_id || !owner.has_rotations ||
      owner.temporal_scheme!=fea::StaggeredHalfKickStart ||
      owner.velocity_phase!=tl::fea::NodalVelocityPhase::Collocated || owner.epoch || owner.time!=0 ||
      owner.velocity_time!=0 || owner.reactions_valid || !std::isfinite(owner.fixed_dt) || owner.fixed_dt<=0 ||
      owner.node_count!=x.node_count || owner.node_count!=weights.global_node_count() ||
      !IsFinite(law.wall_x) || !IsFinite(law.stiffness_per_area) || law.stiffness_per_area<=0 ||
      !IsFinite(law.maximum_penetration) || law.maximum_penetration<=0 ||
      !IsFinite(law.parent_force_error) || law.parent_force_error<=0 ||
      !IsFinite(law.parent_energy_error) || law.parent_energy_error<=0 ||
      !IsFinite(c.exposed_clearance) || c.exposed_clearance<=0)
    return {Code::InvalidInput,"Invalid initial owner, prepared weights or physical declaration"};
  ArenaLayout layout; const auto& limits=c.limits;
  if (!limits.parents || limits.parents>MaxActiveNodalWallDeviceParents || !limits.nodes ||
      limits.nodes>MaxActiveNodalWallDeviceNodes || !limits.global_nodes || limits.global_nodes>MaxActiveNodalWallDeviceNodes ||
      owner.node_count>limits.global_nodes || weights.node_count()>limits.nodes || weights.parent_count()>limits.parents ||
      !c.max_host_bytes || c.max_host_bytes>MaxNodalWallHostBytes ||
      !BuildArenaLayout(weights.parent_count(),weights.node_count(),owner.node_count,c.max_device_bytes,layout) ||
      HostPreparationBytes(layout)>c.max_host_bytes)
    return {Code::ResourceLimit,"Nodal contact exceeds its admitted count or complete arena byte budget"};
  try {
    PreparedModel staged;
    if(!staged.Initialize(layout)) return {Code::ResourceLimit,"Contact host arena allocation failed"};
    auto& next=staged.model(); next.config=c;
    next.node_count=weights.node_count(); next.parent_count=weights.parent_count();
    PlanarWallGeometry wall;
    const auto wall_report=wall.Initialize(source);
    if (wall_report.status!=PlanarContactStatus::Ok || wall.wall_x()!=law.wall_x)
      return {Code::GeometryFailure,"Invalid finite wall or wall-X binding"};
    if (CheckPlanarWallBox(wall,motion,c.exposed_clearance,c.wall_binding_id,
        PlanarWallBoxMode::ConservativeExpansion,&next.coverage).status!=PlanarContactStatus::Ok)
      return {Code::GeometryFailure,"The complete admitted motion envelope is not covered"};
    const auto& faces=wall.faces();
    if (next.query.Initialize(faces.data(),static_cast<unsigned>(faces.size()),wall.tolerance())!=Status::kOk)
      return {Code::GeometryFailure,"Finite wall query preparation failed"};
    for (unsigned f=0;f<faces.size();++f) next.face_ids[f]=faces[f].geometry.face_id;
    for (unsigned p=0;p<next.parent_count;++p) {
      const auto& parent=weights.parent(p);
      if (!((parent.family==NodalWallParentFamily::Q4CenterArea && parent.arity==4) ||
            (parent.family==NodalWallParentFamily::T3Native && parent.arity==3)))
        return {Code::InvalidInput,"Contact parent must have its exact native Q4/4 or T3/3 family and arity",UINT32_MAX,p};
      next.parents[p]=parent;
    }
    for (unsigned i=0;i<next.node_count;++i) {
      const auto node=weights.node(i).node; next.nodes[i]=weights.node(i);
      if (node>=owner.node_count || (masks[node]!=0 && masks[node]!=7) ||
          !IsFinite(inverse[node]) || (masks[node]==7 ? inverse[node]!=0 : inverse[node]<=0))
        return {Code::InvalidMass,"Incident masks/mass must be free XYZ or fully fixed",node};
      next.initial_position[node]=x.at(node); next.inverse_mass[node]=inverse[node]; next.fixed[node]=masks[node]==7;
      if (!Inside(x.at(node),next.coverage.physical))
        return {Code::GeometryFailure,"An initial incident node is outside the admitted motion envelope",node};
      double rate=0;
      const LumpedTranslationMassView mass{next.inverse_mass,next.fixed,static_cast<std::uint32_t>(owner.node_count),0,
                                         TranslationMassModel::kIsotropicLumped};
      for (unsigned p=0;p<next.parent_count;++p) for (unsigned l=0;l<next.parents[p].arity;++l) {
        if (next.parents[p].nodes[l]!=node) continue;
        NodalWallPointResult value;
        const auto report=EvaluateNodalWallPoint({node,next.parents[p].share},x.at(node),{},mass,law,1,&value);
        if (report.status!=NodalWallStatus::Ok)
          return {Code::PointFailure,"Initial node or all-active rate is not admissible",node,p,report};
        if (!value.fixed && !AddUpper(rate,value.row.stiffness[0],&rate))
          return {Code::NonFiniteArithmetic,"Assembled contact rate overflows",node,p};
      }
      if (rate>next.rate) next.rate=rate;
    }
    next.prepared=true; *output=std::move(staged);
    return {Code::Ok,"OK"};
  } catch (const std::bad_alloc&) {
    return {Code::ResourceLimit,"Bounded host wall preparation allocation failed"};
  }
}
} // namespace tlfea::contact::nodal_wall_device_detail

namespace tlfea::contact {
NodalWallContactDevice::NodalWallContactDevice()=default;
NodalWallContactDevice::~NodalWallContactDevice()=default;
NodalWallContactDevice::Impl::~Impl() { if (device) cudaFree(device); }
NodalWallDeviceReport NodalWallContactDevice::Initialize(const NodalWallDeviceConfig& config,PlanarWallView wall,
    const NodalWallWeights& weights,VectorView x,const double* inverse,const std::uint8_t* masks,PlanarWallBox motion) {
  using namespace nodal_wall_device_detail;
  if (impl_) return {Code::InvalidInput,"Nodal wall contributor is already initialized"};
  try {
    PreparedModel storage;
    auto report=PrepareModel(config,wall,weights,x,inverse,masks,motion,&storage);
    if (report.status!=Code::Ok) return report;
    auto next=std::make_unique<Impl>(); next->config=config; next->rate=storage.model().rate;
    next->prepared=std::move(storage); const auto bytes=next->prepared.layout().bytes;
    report=next->Check(cudaGetLastError()); if (report.status!=Code::Ok) return report;
    report=next->Check(cudaMalloc(reinterpret_cast<void**>(&next->device),bytes));
    if (report.status!=Code::Ok) return report;
    next->device_shadow=next->prepared.Rebase(next->device);
    report=next->Check(cudaMemcpy(next->device,next->prepared.data(),bytes,cudaMemcpyHostToDevice));
    if (report.status!=Code::Ok) return report;
    report=next->Check(cudaMemcpy(next->device,&next->device_shadow,sizeof(Storage),cudaMemcpyHostToDevice));
    if (report.status!=Code::Ok) return report;
    impl_=std::move(next); return {Code::Ok,"Nodal wall contributor initialized"};
  } catch (const std::bad_alloc&) { return {Code::ResourceLimit,"Host contact staging allocation failed"}; }
}
void NodalWallContactDevice::DiscardTrial() noexcept {
  if (impl_) { impl_->has_base=false; impl_->has_results=false; }
}
tl::fea::NodalAllocationInfo NodalWallContactDevice::allocations() const noexcept {
  return impl_ ? tl::fea::NodalAllocationInfo{impl_->prepared.layout().bytes,1} : tl::fea::NodalAllocationInfo{};
}
double NodalWallContactDevice::stiffness_rate_bound() const noexcept { return impl_?impl_->rate:0; }
} // namespace tlfea::contact
