// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Planning.h"
#include "InitialSeed.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <new>
#include <type_traits>
namespace tlfea::contact::radioss_type25 {
namespace fe=tl::fea;namespace rd=runtime_detail;
namespace {
TransactionReport Error(TransactionStatus s,const char* message){return {s,message};}
}
Transaction::Transaction()=default;Transaction::~Transaction()=default;
Transaction::Impl::~Impl(){if(stream)cudaStreamSynchronize(stream);activity.reset();if(arena)cudaFree(arena);}
template<class Source>
TransactionReport Transaction::InitializeSource(const TransactionConfig& config,const Source& input_source,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,const fe::ShellPhysicalPublicationIdentity& identity,
    TransactionLimits limits,const initial_source::PreparedSource* prepared,const startup::FixedMainView* ready) noexcept try {
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
  rd::Plan plan(limits);
  Source source=input_source;TransactionReport status;
  if(prepared) {
    GeneralTransactionForecast forecast;
    if constexpr(std::is_same_v<Source,FixedMainSource>) {
      if(!ready)return Error(TransactionStatus::SourceMismatch,"General fixed source requires genuine ready phase");
      status=rd::PrepareGeneralPlan(config,input_source,*ready,*prepared,physical,limits,sizeof(Transaction)+sizeof(Impl),plan,source,forecast);
    } else {
      if(ready)return Error(TransactionStatus::SourceMismatch,"Moving source cannot import fixed-ready cache");
      status=rd::PrepareGeneralPlan(config,input_source,*prepared,physical,limits,sizeof(Transaction)+sizeof(Impl),plan,source,forecast);
    }
  } else status=rd::PreparePlan(config,source,physical,limits,sizeof(Transaction)+sizeof(Impl),plan);
  if(status.status!=TransactionStatus::Ok)return status;
  cudaStream_t stream=nullptr;
  const auto borrowed=owner.BorrowOwnerStream(&stream);
  if(borrowed.status!=fe::NodalStatus::Ok)return Error(TransactionStatus::OwnerFailure,borrowed.message);
  auto& upload=plan.upload;const auto& forecast=plan.forecast;const auto& layout=plan.layout;
  const auto& normal=plan.normal;const auto& incidence_limits=plan.incidence_limits;
  const auto& readback_layout=plan.readback;const auto& rows=plan.rows;const auto& secondary=plan.secondary;
  auto next=std::make_unique<Impl>(physical);next->owner=&owner;next->publication=&publication;
  next->participants=participants;next->identity=identity;next->config=config;next->source=source;
  next->limits=limits;next->forecast=forecast;next->layout=layout;next->stream=stream;
  if(config.activity==ContactActivityPolicy::AllActivePrefix) {
    const auto checked=next->active_prefix.Initialize(owner,publication,physical,participants,identity,{limits.max_host_bytes});
    if(checked.status!=fe::ActivePrefixStatus::Ok)
      return Error(checked.status==fe::ActivePrefixStatus::InactiveParent?TransactionStatus::ActivityChange:
        TransactionStatus::PublicationFailure,checked.message);
  }
  if(!units_detail::Make(config.units,next->units))return Error(TransactionStatus::InvalidInput,"Invalid native units");
  if(!next->readback.Initialize(readback_layout.bytes())||!next->readback.Construct<NativeGeometryHistory>(rows)||
     !next->readback.Construct<lifecycle::Secondary>(secondary))return Error(TransactionStatus::ResourceLimit,"Native readback allocation failed");
  next->readback_rows=rows;next->readback_secondary=secondary;
  auto error=cudaGetLastError();if(error!=cudaSuccess)return Error(TransactionStatus::DeviceFailure,"Pending CUDA error at initialization");
  error=cudaMalloc(&next->arena,layout.bytes);if(error!=cudaSuccess)return Error(TransactionStatus::DeviceFailure,"Native runtime allocation failed");
  next->device=rd::Bind(next->arena,layout,source,limits,normal);
  const auto copy=[&](const void* values,const tl::util::ArenaRegion& region,std::size_t bytes=SIZE_MAX) {
    const auto count=bytes==SIZE_MAX?region.bytes:bytes;
    if(count>region.bytes){error=cudaErrorInvalidValue;return;}
    if(error==cudaSuccess&&count)error=cudaMemcpyAsync(tl::util::ArenaPointer<std::byte>(next->arena,region),values,
        count,cudaMemcpyHostToDevice,stream);
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
    if(normal.mixed)copy(upload.moving.starter.primary_to_partner,layout.normal.partners);
    copy(upload.moving.main_coefficients.data(),layout.normal.coefficients);
    copy(upload.moving.free_main_ids.data(),layout.normal.free_mains,normal.free_count*sizeof(std::uint32_t));
    for(unsigned slab=0;slab<2;++slab) {
      copy(upload.moving.starter.starter.face_normals,layout.normal.face[slab]);
      copy(upload.moving.starter.starter.references,layout.normal.references[slab]);
    }
  }
  const auto drained=cudaStreamSynchronize(stream);
  if(error!=cudaSuccess||drained!=cudaSuccess)return Error(TransactionStatus::DeviceFailure,"Native source upload failed");
  if(prepared) {
    // Only the primary arena is needed for this handoff. Upload drains both
    // history/flag slabs and final corners, then retires its private seed
    // before any paired inventory, maintenance or incidence allocation.
    const auto seeded=rd::InitialSeedAccess::Upload(*prepared,next->device,stream,next->initialization);
    if(seeded.status!=TransactionStatus::Ok)return seeded;
  }
  if(config.activity==ContactActivityPolicy::ShellRemoval) {
    next->activity=std::make_unique<rd::ActivityRuntime>();
    auto checked=rd::ActivityReport(next->activity->snapshot.Initialize(owner,publication,physical,
        participants,identity,plan.activity.snapshot_limits));
    if(checked.status!=TransactionStatus::Ok)return checked;
    checked=rd::ActivityReport(next->activity->snapshot.ValidateType45Source(source.activity_type45));
    if(checked.status!=TransactionStatus::Ok)return checked;
    checked=next->activity->operands.Initialize(plan.activity.source,source,
        normal.enabled?&upload.moving.topology:nullptr,config.units,
        rd::ActivitySlot(next->arena,layout,source,normal),stream,plan.activity.operand_limits);
    if(checked.status!=TransactionStatus::Ok)return checked;
  }
  for(unsigned slab=0;slab<2;++slab) {
    if(next->inventory[slab].Initialize(upload.inventory,limits.inventory,stream)!=candidates::Status::Ok||
       next->maintenance[slab].Initialize(upload.maintenance,limits.maintenance,stream)!=search::Status::Ok)
      return Error(TransactionStatus::DeviceFailure,"Native paired workspace initialization failed");
  }
  if(next->incidence.Initialize(incidence_limits,stream)!=assembly::IncidenceStatus::Ok)
    return Error(TransactionStatus::DeviceFailure,"Native incidence initialization failed");
  next->source.selection=next->device.source;next->source.primary_parent_ids=nullptr;next->source.primary_curvature=nullptr;
  next->source.activity_controls=nullptr;next->source.activity_type45=nullptr;
  if(!next->state.Attach(owner,source.source_id,next->issuer,bool(next->activity)))return Error(TransactionStatus::PublicationFailure,"Native participant attachment rejected");
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
TransactionReport Transaction::Initialize(const TransactionConfig& config,const MixedMovingMainSource& source,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,const fe::ShellPhysicalPublicationIdentity& identity,
    TransactionLimits limits) noexcept {
  return InitializeSource(config,source,owner,publication,physical,participants,identity,limits);
}
TransactionReport Transaction::GeneralInitialize(const TransactionConfig& config,const FixedMainSource& source,
    const startup::FixedMainView& ready,const initial_source::PreparedSource& prepared,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,const fe::ShellPhysicalPublicationIdentity& identity,
    TransactionLimits limits) noexcept {
  return InitializeSource(config,source,owner,publication,physical,participants,identity,limits,&prepared,&ready);
}
TransactionReport Transaction::GeneralInitialize(const TransactionConfig& config,const MovingMainSource& source,
    const initial_source::PreparedSource& prepared,fe::FENodalState& owner,fe::ShellBatchPublication& publication,
    const fe::ShellPhysicalBinding& physical,const fe::ShellPhysicalParticipants& participants,
    const fe::ShellPhysicalPublicationIdentity& identity,TransactionLimits limits) noexcept {
  return InitializeSource(config,source,owner,publication,physical,participants,identity,limits,&prepared,nullptr);
}
TransactionReport Transaction::GeneralInitialize(const TransactionConfig& config,const MixedMovingMainSource& source,
    const initial_source::PreparedSource& prepared,fe::FENodalState& owner,fe::ShellBatchPublication& publication,
    const fe::ShellPhysicalBinding& physical,const fe::ShellPhysicalParticipants& participants,
    const fe::ShellPhysicalPublicationIdentity& identity,TransactionLimits limits) noexcept {
  return InitializeSource(config,source,owner,publication,physical,participants,identity,limits,&prepared,nullptr);
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
TransactionInitializationDiagnostics Transaction::initialization_diagnostics() const noexcept {
  return impl_?impl_->initialization:TransactionInitializationDiagnostics{};
}
} // namespace tlfea::contact::radioss_type25
