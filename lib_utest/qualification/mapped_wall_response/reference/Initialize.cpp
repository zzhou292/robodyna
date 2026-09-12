// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <new>
namespace tlfea::contact {
namespace m=nodal_wall_mapped;
namespace d=nodal_wall_device_detail;
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
NodalWallMappedContact::NodalWallMappedContact()=default;
NodalWallMappedContact::~NodalWallMappedContact()=default;
NodalWallMappedContact::Impl::Impl(const NodalWallMappedSource& source)
    :physical(*source.physical),rigid(*source.rigid),publication(source.publication),
     participants(source.participants),identity(source.identity),witnesses(source.cin.witness_count) {}
NodalWallMappedContact::Impl::~Impl() {
  if(device_sidecar) cudaFree(device_sidecar);
  if(device) cudaFree(device);
}
NodalWallDeviceReport NodalWallMappedContact::Initialize(const NodalWallDeviceConfig& config,
    PlanarWallView wall,const NodalWallWeights& weights,const NodalWallMappedSource& source,
    fe::FENodalState& owner,PlanarWallBox motion,NodalWallMappedLimits limits) {
  if(impl_) return {Code::InvalidInput,"Mapped wall already initialized"};
  d::ArenaLayout arena;
  m::Layout layout;
  fe::shell_physical_owner::ProofLayout proof;
  fe::ShellMappedFootprint forecast;
  auto report=m::Preflight(config,weights,source,limits,sizeof(NodalWallMappedContact)+sizeof(Impl),arena,layout,proof,forecast);
  if(report.status!=Code::Ok) return report;
  if(!fe::trial_identity::SameStamp(owner.accepted(),config.owner) ||
      owner.ValidateRigidAssemblyBinding(*source.rigid).status!=fe::NodalStatus::Ok ||
      source.publication->ValidatePhysicalSources(owner,*source.physical,source.participants,
          source.identity).status!=fe::ShellPublicationStatus::Success)
    return {Code::WrongOwner,"Mapped contact requires the actual complete physical publication and owner"};
  const auto authenticated=fe::shell_physical_owner::AuthenticateInitial(*source.physical->coefficients(),
      owner,config.owner,source.identity.startup,source.cin,proof);
  if(authenticated.status!=fe::NodalStatus::Ok)
    return {authenticated.status==fe::NodalStatus::DeviceFailure?Code::DeviceFailure:Code::WrongOwner,authenticated.message};
  try {
    auto next=std::make_unique<Impl>(source);
    next->owner=&owner;
    next->config=config;
    next->forecast=forecast;
    next->layout=layout;
    if(!next->host.Initialize(layout.bytes)) return {Code::ResourceLimit,"Mapped contact host sidecar allocation failed"};
    next->host.Construct<std::uint8_t>(layout.accepted);
    next->host.Construct<std::uint8_t>(layout.proposed);
    next->host.Construct<std::uint32_t>(layout.roots);
    next->host.Construct<RigidContactBody>(layout.bodies);
    next->host.Construct<double>(layout.traces);
    next->host.Construct<double>(layout.stiffness);
    next->host.Construct<double>(layout.inverse);
    next->host.Construct<m::Summary>(layout.summary);
    next->host.Construct<m::ObserverSummary>(layout.observer);
    next->host.Construct<m::IntervalSummary>(layout.interval);
    next->local=m::Bind(next->host.data(),layout);
    tl::util::BoundedStartupArray<double,0> coordinates;
    coordinates.Resize(3*config.owner.node_count);
    for(std::size_t n=0;n<config.owner.node_count;++n) {
      const auto x=source.physical->domain()->nodes()[n].position;
      coordinates[3*n]=x.x;
      coordinates[3*n+1]=x.y;
      coordinates[3*n+2]=x.z;
    }
    report=next->PrepareSources(weights,source.cin,coordinates.data());
    if(report.status!=Code::Ok) return report;
    report=d::PreparePhysicalModel(config,wall,weights,
        {coordinates.data(),weights.global_node_count(),3,1},motion,&next->prepared);
    if(report.status!=Code::Ok) return report;
    // Actual initial activity is obtained from the already initialized common
    // owner/typed histories, including virgin flags; no default 'all one' seed.
    report=next->CaptureActivity(nullptr,nullptr);
    if(report.status!=Code::Ok) return report;
    report=next->CaptureBodies();
    if(report.status!=Code::Ok) return report;
    report=next->Check(cudaGetLastError());
    if(report.status!=Code::Ok) return report;
    report=next->Check(cudaMalloc(reinterpret_cast<void**>(&next->device),arena.bytes));
    if(report.status!=Code::Ok) return report;
    report=next->Check(cudaMalloc(&next->device_sidecar,layout.bytes));
    if(report.status!=Code::Ok) return report;
    next->shadow=next->prepared.Rebase(next->device);
    next->remote=m::Bind(next->device_sidecar,layout);
    report=next->Check(cudaMemcpy(next->device,next->prepared.data(),arena.bytes,cudaMemcpyHostToDevice));
    if(report.status!=Code::Ok) return report;
    report=next->Check(cudaMemcpy(next->device,&next->shadow,sizeof(d::Storage),cudaMemcpyHostToDevice));
    if(report.status!=Code::Ok) return report;
    report=next->Check(cudaMemcpy(next->device_sidecar,next->host.data(),layout.bytes,cudaMemcpyHostToDevice));
    if(report.status!=Code::Ok) return report;
    impl_=std::move(next);
    return {Code::Ok,"Physical finite-wall contributor initialized without a second clock"};
  } catch(const std::bad_alloc&) {
    return {Code::ResourceLimit,"Mapped contact startup allocation failed"};
  }
}
void NodalWallMappedContact::DiscardTrial() noexcept {
  if(impl_) {
    impl_->has_base=false;
    impl_->has_results=false;
  }
}
fe::NodalAllocationInfo NodalWallMappedContact::allocations() const noexcept {
  return impl_?fe::NodalAllocationInfo{impl_->forecast.device_bytes,2}:fe::NodalAllocationInfo{};
}
} // namespace tlfea::contact
