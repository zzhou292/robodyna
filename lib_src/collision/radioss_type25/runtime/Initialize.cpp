// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <new>
namespace tlfea::contact::radioss_type25 {
namespace fe=tl::fea;namespace rd=runtime_detail;
namespace {
TransactionReport Error(TransactionStatus s,const char* message){return {s,message};}
bool Add(std::size_t value,std::size_t& sum){if(value>SIZE_MAX-sum)return false;sum+=value;return true;}
}
Transaction::Transaction()=default;Transaction::~Transaction()=default;
Transaction::Impl::~Impl(){if(stream)cudaStreamSynchronize(stream);if(arena)cudaFree(arena);}
template<class Source>
TransactionReport Transaction::InitializeSource(const TransactionConfig& config,const Source& source,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,const fe::ShellPhysicalPublicationIdentity& identity,
    TransactionLimits limits) noexcept try {
  if(impl_)return Error(TransactionStatus::AlreadyInitialized,"Native contact transaction is immutable");
  const auto stamp=owner.accepted();
  if(!stamp.owner_id||stamp.epoch||stamp.time!=0||!stamp.has_rotations||
     stamp.temporal_scheme!=fe::NodalTemporalScheme::StaggeredHalfKickStart||
     stamp.velocity_phase!=fe::NodalVelocityPhase::Collocated||stamp.reactions_valid||
     (config.response_mass==ResponseMassPolicy::StaticPhysicalLedger&&stamp.rigid_groups.group_count))
    return Error(TransactionStatus::UnsupportedProfile,"Fresh conventional fixed-step staggered owner required");
  const auto authenticated=publication.ValidatePhysicalSources(owner,physical,participants,identity);
  if(authenticated.status!=fe::ShellPublicationStatus::Success)
    return Error(TransactionStatus::PublicationFailure,authenticated.message);
  rd::SourceStaging upload;auto status=rd::PrepareSource(config,source,physical,limits,upload);
  if(status.status!=TransactionStatus::Ok)return status;
  cudaStream_t stream=nullptr;
  const auto borrowed=owner.BorrowOwnerStream(&stream);
  if(borrowed.status!=fe::NodalStatus::Ok)return Error(TransactionStatus::OwnerFailure,borrowed.message);
  candidates::Forecast inventory;search::Forecast maintenance;assembly::IncidenceForecast incidence;
  const auto candidates_status=candidates::Inventory::Preflight(upload.inventory,limits.inventory,inventory);
  if(candidates_status!=candidates::Status::Ok)return Error(candidates_status==candidates::Status::DeviceFailure?
      TransactionStatus::DeviceFailure:TransactionStatus::ResourceLimit,"Native candidate preflight rejected");
  if(search::Maintenance::Preflight(upload.maintenance,limits.maintenance,maintenance)!=search::Status::Ok)
    return Error(TransactionStatus::ResourceLimit,"Native maintenance preflight rejected");
  assembly::IncidenceLimits incidence_limits{limits.optimized_candidates,source.selection.node_count,
    limits.optimized_candidates/source.force_packet_size+(limits.optimized_candidates%source.force_packet_size!=0),limits.max_device_bytes};
  if(assembly::DeviceIncidenceBuilder::Preflight(incidence_limits,incidence)!=assembly::IncidenceStatus::Ok)
    return Error(TransactionStatus::ResourceLimit,"Native ASS0 incidence preflight rejected");
  std::size_t cub=0;
  if(rd::QueryScratch(source.selection.secondary_count,limits.optimized_candidates,cub)!=cudaSuccess)
    return Error(TransactionStatus::DeviceFailure,"Native runtime scratch query failed");
  const rd::NormalShape normal{upload.moving.enabled,upload.moving.free_main_ids.size(),upload.moving.activation};
  rd::Layout layout;if(!rd::MakeLayout(source,limits,cub,layout,normal,config.response_mass))return Error(TransactionStatus::ResourceLimit,"Native runtime arena exceeds cap");
  TransactionForecast forecast;forecast.raw_pair_capacity=limits.inventory.max_pairs;
  forecast.optimized_capacity=limits.optimized_candidates;forecast.sliding_capacity=limits.sliding_entries;
  forecast.runtime_device_bytes=layout.bytes;forecast.normal_device_bytes=layout.normal.bytes;
  if(!Add(inventory.device_bytes,forecast.inventory_device_bytes)||!Add(inventory.device_bytes,forecast.inventory_device_bytes)||
     !Add(maintenance.device_bytes,forecast.maintenance_device_bytes)||!Add(maintenance.device_bytes,forecast.maintenance_device_bytes))
    return Error(TransactionStatus::ResourceLimit,"Native paired arena forecast overflow");
  forecast.incidence_device_bytes=incidence.device_bytes;
  if(!Add(layout.bytes,forecast.device_bytes)||!Add(forecast.inventory_device_bytes,forecast.device_bytes)||
     !Add(forecast.maintenance_device_bytes,forecast.device_bytes)||!Add(incidence.device_bytes,forecast.device_bytes)||
     forecast.device_bytes>limits.max_device_bytes)
    return Error(TransactionStatus::ResourceLimit,"Complete native device forecast exceeds cap");
  tl::util::BoundedArenaLayout readback_layout(limits.max_host_bytes);tl::util::ArenaRegion rows,secondary;
  if(!readback_layout.Append<NativeGeometryHistory>(source.selection.secondary_count,rows)||
     !readback_layout.Append<lifecycle::Secondary>(source.selection.secondary_count,secondary))
    return Error(TransactionStatus::ResourceLimit,"Native readback forecast exceeds cap");
  forecast.host_bytes=sizeof(Transaction)+sizeof(Impl);
  if(!Add(readback_layout.bytes(),forecast.host_bytes)||!Add(inventory.startup_host_bytes,forecast.host_bytes)||!Add(inventory.startup_host_bytes,forecast.host_bytes)||
     !Add(maintenance.startup_host_bytes,forecast.host_bytes)||!Add(maintenance.startup_host_bytes,forecast.host_bytes)||!Add(incidence.host_bytes,forecast.host_bytes)||
     !Add(forecast.host_bytes,forecast.startup_host_bytes)||!Add(upload.bytes+sizeof(upload),forecast.startup_host_bytes)||
     forecast.startup_host_bytes>limits.max_host_bytes)
    return Error(TransactionStatus::ResourceLimit,"Complete native host forecast exceeds cap");
  auto next=std::make_unique<Impl>(physical);next->owner=&owner;next->publication=&publication;
  next->participants=participants;next->identity=identity;next->config=config;next->source=source;
  next->limits=limits;next->forecast=forecast;next->layout=layout;next->stream=stream;
  if(!units_detail::Make(config.units,next->units))return Error(TransactionStatus::InvalidInput,"Invalid native units");
  if(!next->readback.Initialize(readback_layout.bytes())||!next->readback.Construct<NativeGeometryHistory>(rows)||
     !next->readback.Construct<lifecycle::Secondary>(secondary))return Error(TransactionStatus::ResourceLimit,"Native readback allocation failed");
  next->readback_rows=rows;next->readback_secondary=secondary;
  auto error=cudaGetLastError();if(error!=cudaSuccess)return Error(TransactionStatus::DeviceFailure,"Pending CUDA error at initialization");
  error=cudaMalloc(&next->arena,layout.bytes);if(error!=cudaSuccess)return Error(TransactionStatus::DeviceFailure,"Native runtime allocation failed");
  next->device=rd::Bind(next->arena,layout,source,limits,normal);
  const auto copy=[&](const void* values,const tl::util::ArenaRegion& region) {
    if(error==cudaSuccess&&region.bytes)error=cudaMemcpyAsync(tl::util::ArenaPointer<std::byte>(next->arena,region),values,
        region.bytes,cudaMemcpyHostToDevice,stream);
  };
  const auto& s=source.selection;
  copy(s.nodes,layout.nodes);copy(s.mains,layout.mains);copy(s.normals,layout.normals);
  copy(s.normal_to_main.offsets,layout.normal_offsets);copy(s.normal_to_main.entries,layout.normal_entries);
  copy(s.removed_main_by_secondary.offsets,layout.removed_offsets);copy(s.removed_main_by_secondary.entries,layout.removed_entries);
  for(unsigned slab=0;slab<2;++slab){copy(s.secondary,layout.secondary[slab]);copy(upload.history.data(),layout.history[slab]);}
  copy(upload.positions.data(),layout.reference_positions);copy(upload.native_mass.data(),layout.native_mass);
  copy(upload.secondary_stiffness.data(),layout.secondary_stiffness);copy(upload.secondary_gaps.data(),layout.secondary_gaps);
  copy(upload.main_stiffness.data(),layout.main_stiffness);copy(upload.main_gaps.data(),layout.main_gaps);copy(upload.main_curvature.data(),layout.main_curvature);
  if(normal.enabled) {
    copy(upload.moving.topology.mains,layout.normal.topology);
    copy(upload.moving.main_coefficients.data(),layout.normal.coefficients);
    copy(upload.moving.free_main_ids.data(),layout.normal.free_mains);
    for(unsigned slab=0;slab<2;++slab) {
      copy(upload.moving.starter.starter.face_normals,layout.normal.face[slab]);
      copy(upload.moving.starter.starter.references,layout.normal.references[slab]);
    }
  }
  const auto drained=cudaStreamSynchronize(stream);
  if(error!=cudaSuccess||drained!=cudaSuccess)return Error(TransactionStatus::DeviceFailure,"Native source upload failed");
  for(unsigned slab=0;slab<2;++slab) {
    if(next->inventory[slab].Initialize(upload.inventory,limits.inventory,stream)!=candidates::Status::Ok||
       next->maintenance[slab].Initialize(upload.maintenance,limits.maintenance,stream)!=search::Status::Ok)
      return Error(TransactionStatus::DeviceFailure,"Native paired workspace initialization failed");
  }
  if(next->incidence.Initialize(incidence_limits,stream)!=assembly::IncidenceStatus::Ok)
    return Error(TransactionStatus::DeviceFailure,"Native incidence initialization failed");
  next->source.selection=next->device.source;next->source.primary_parent_ids=nullptr;next->source.primary_curvature=nullptr;
  if(!next->state.Attach(owner,source.source_id,next->issuer))return Error(TransactionStatus::PublicationFailure,"Native participant attachment rejected");
  impl_=std::move(next);return {TransactionStatus::Ok,"OK"};
} catch(const std::bad_alloc&){return Error(TransactionStatus::ResourceLimit,"Native startup allocation failed");}
TransactionReport Transaction::Initialize(const TransactionConfig& config,const FixedMainSource& source,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,const fe::ShellPhysicalPublicationIdentity& identity,
    TransactionLimits limits) noexcept {
  return InitializeSource(config,source,owner,publication,physical,participants,identity,limits);
}
TransactionReport Transaction::Initialize(const TransactionConfig& config,const MovingMainSource& source,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,const fe::ShellPhysicalPublicationIdentity& identity,
    TransactionLimits limits) noexcept {
  return InitializeSource(config,source,owner,publication,physical,participants,identity,limits);
}
fe::ShellPhysicalScratchRosterEntry Transaction::roster_entry() noexcept {
  return impl_?fe::ShellPhysicalScratchRosterEntry{&impl_->issuer,impl_->source.source_id}:fe::ShellPhysicalScratchRosterEntry{};
}
fe::NativeContactRosterEntry Transaction::native_roster_entry() noexcept {
  return impl_ ? fe::NativeContactRosterEntry{&impl_->issuer, impl_->source.source_id}
               : fe::NativeContactRosterEntry{};
}
TransactionSourceInfo Transaction::source_info() const noexcept {
  if(!impl_)return {};const auto& s=impl_->source;
  return {s.source_id,s.topology_generation,s.selection.generation,s.selection.node_count,
    s.selection.secondary_count,s.primary_main_count,s.selection.main_count,true};
}
TransactionForecast Transaction::allocations() const noexcept{return impl_?impl_->forecast:TransactionForecast{};}
TransactionDiagnostics Transaction::last_diagnostics() const noexcept{return impl_?impl_->diagnostics:TransactionDiagnostics{};}
} // namespace tlfea::contact::radioss_type25
