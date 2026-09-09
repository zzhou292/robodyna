#include "Q4PlanarContactStorage.h"

#include <cmath>
#include <new>

namespace tlfea::contact {
namespace {
using Code=Q4PlanarContactStatus;
bool ValidLimits(const Q4IntegrationLimits& l) {
  return l.max_leaves && l.max_leaves <= MaxQ4IntegrationLeaves && l.max_depth <= MaxQ4IntegrationDepth &&
         l.max_visited && l.max_visited <= MaxQ4IntegrationVisits &&
         std::isfinite(l.force_error) && l.force_error > 0 && std::isfinite(l.energy_error) && l.energy_error > 0;
}
}
Q4PlanarContact::Q4PlanarContact() = default;
Q4PlanarContact::~Q4PlanarContact() = default;
Q4PlanarContact::Impl::~Impl() { if (device) cudaFree(device); }

Q4PlanarContactReport Q4PlanarContact::Initialize(const Q4PlanarContactConfig& config,PlanarWallView source_wall,
                                                const Q4SurfaceView& surface,const Q4FixedYZMassView& mass) {
  if (impl_) return {Code::InvalidInput,"Q4 contact already initialized"};
  if (!q4_planar_detail::ValidBackend(config.integration_backend) ||
      !config.owner.owner_id || !config.configuration_id || !config.wall_binding_id ||
      !std::isfinite(config.owner.time) || !std::isfinite(config.owner.fixed_dt) || config.owner.fixed_dt <= 0 ||
      !std::isfinite(config.stiffness_per_area) || config.stiffness_per_area <= 0 ||
      !std::isfinite(config.maximum_penetration) || config.maximum_penetration <= 0 || !ValidLimits(config.integration) ||
      config.owner.node_count != surface.positions.node_count || mass.base_epoch != config.owner.epoch)
    return {Code::InvalidInput,"Invalid backend, owner, contact parameters, integration limits or reference association"};
  const auto device_bytes=q4_planar_detail::DeviceBytes(config.integration_backend);
  if (!config.owner.node_count || config.owner.node_count > tl::fea::MaxTranslationNodes ||
      !config.max_device_bytes || config.max_device_bytes > MaxPlanarContactDeviceBytes ||
      device_bytes > config.max_device_bytes)
    return {Code::ResourceLimit,"Q4 contact exceeds the node or complete device storage budget"};
  try {
    auto candidate=std::make_unique<Impl>();
    auto staged=std::make_unique<q4_planar_detail::Storage>();
    auto geometry=candidate->wall.Initialize(source_wall);
    if (geometry.status != PlanarContactStatus::Ok)
      return {Code::GeometryFailure,geometry.message,UINT32_MAX,UINT32_MAX,geometry.status};
    Q4PlanarGeometry prepared;
    geometry=prepared.Initialize(candidate->wall,surface,mass,config.exposed_clearance);
    if (geometry.status != PlanarContactStatus::Ok)
      return {Code::GeometryFailure,geometry.message,geometry.sample,UINT32_MAX,geometry.status};
    auto& model=staged->model;
    model.config=config; model.parent_count=surface.parent_count;
    model.wall_x=candidate->wall.wall_x(); model.wall_tolerance=candidate->wall.tolerance();
    for (auto& fixed:model.fixed) fixed=7;
    const auto reference=prepared.view();
    for (unsigned p=0;p<model.parent_count;++p) {
      model.reference[p]=reference.parents[p]; model.parents[p]=reference.parents[p].parent;
      for (auto node:model.parents[p].nodes) {
        model.inverse_mass[node]=mass.inverse_mass[node]; model.fixed[node]=mass.translation_fixed_bits[node];
      }
    }
    const auto stiffness=BuildQ4PlanarStiffness(reference,mass,config.stiffness_per_area,&model.stiffness);
    if (stiffness != PlanarContactStatus::Ok)
      return {Code::InvalidMass,"Q4 all-active stiffness/mass bound is not representable",UINT32_MAX,UINT32_MAX,stiffness};
    candidate->config=config; candidate->parent_count=model.parent_count; candidate->rate_bound=model.stiffness.rate_bound;
    candidate->device_bytes=device_bytes;
    auto report=candidate->Check(cudaMalloc(reinterpret_cast<void**>(&candidate->device),device_bytes));
    if (report.status != Code::Ok) return report;
    staged->leaves=reinterpret_cast<unsigned char*>(candidate->device)+sizeof(*staged);
    report=candidate->Check(cudaMemcpy(candidate->device,staged.get(),sizeof(*staged),cudaMemcpyHostToDevice));
    if (report.status != Code::Ok) return report;
    report=candidate->ConstructLeaves();
    if (report.status != Code::Ok) return report;
    impl_=std::move(candidate);
    return {Code::Ok,"Q4 contact initialized"};
  } catch (const std::bad_alloc&) {
    return {Code::ResourceLimit,"Q4 contact host staging allocation failed"};
  }
}

tl::fea::NodalAllocationInfo Q4PlanarContact::allocations() const noexcept {
  return impl_ ? tl::fea::NodalAllocationInfo{impl_->device_bytes,1} : tl::fea::NodalAllocationInfo{};
}
double Q4PlanarContact::stiffness_rate_bound() const noexcept { return impl_ ? impl_->rate_bound : 0; }
}  // namespace tlfea::contact
