#include "SourcePartElasticWallInternal.h"
#include <cstring>
#include <new>

namespace crash::cases::source_part_elastic {
namespace {
Report Convert(const source_part_wall::SourcePartWallReport& r) {
    if(r) return Success();
    return Failure(r.status==source_part_wall::SourcePartWallStatus::DeviceFailure?Status::DeviceFailure:Status::ComponentFailure,
        r.message);
}
bool Identity(const wall_contact::NodalWallDiagnostics& d,const Config& c,
              const source_part_wall::SourcePartWallSetup& setup,const fe::NodalStamp& base,std::uint64_t attempt) {
    return d.valid&&d.owner_id==base.owner_id&&d.configuration_id==c.configuration_id&&
        d.qualification_id==c.qualification_id&&d.wall_binding_id==setup.settings()->wall_binding_id&&
        d.base_epoch==base.epoch&&d.attempt==attempt&&d.node_count==NodeCount&&
        d.parent_count==source::Q4Count+source::T3Count&&d.scheme==base.temporal_scheme&&
        d.base_time==base.time&&d.base_velocity_time==base.velocity_time;
}
}
Report SourcePartElasticCase::Initialize(const source::SourcePartContactFixture& source,const Config& config,
    const case_data::CanonicalWall& canonical,const std::string& bytes,const source_part_wall::SourcePartWallSettings& settings) {
    if(impl_->initialized) return Failure(Status::AlreadyInitialized,"Source-part case is immutable after initialization");
    if(!source.prepared()||!ValidConfig(config)||config.experiment!=Experiment::MeshWallImpact||
       settings.configuration_id!=config.configuration_id||settings.qualification_id!=config.qualification_id||
       std::memcmp(settings.initial_velocity.data(),config.initial_velocity.data(),sizeof(config.initial_velocity)))
        return Failure(Status::InvalidInput,"Wall configuration differs from the explicit source startup declaration");
    std::unique_ptr<Impl> staged(new(std::nothrow) Impl);
    if(!staged) return Failure(Status::ComponentFailure,"Source-part startup storage allocation failed");
    auto report=staged->Initialize(source,config);
    if(report) report=staged->InitializeWall(canonical,bytes,settings);
    if(!report) return report;
    impl_.swap(staged);
    return Success();
}
Report SourcePartElasticCase::Impl::InitializeWall(const case_data::CanonicalWall& canonical,const std::string& bytes,
    const source_part_wall::SourcePartWallSettings& settings) {
    std::unique_ptr<SourcePartWallState> staged(new(std::nothrow) SourcePartWallState);
    if(!staged) return Failure(Status::ComponentFailure,"Source wall state allocation failed");
    const auto report=staged->contributor.Initialize(source,binding,accepted.stamp,inverse_mass.data(),
        initial_kinetic,canonical,bytes,settings);
    if(!report) return Convert(report);
    staged->accepted_metrics.carried_angular_momentum=CarriedAngularMomentum(binding,accepted);
    // No trial or invented contact result at epoch zero. The actual immutable
    // placement certificate proves separation; nodal/material history is rest.
    wall=std::move(staged);
    return Success();
}
Report SourcePartElasticCase::Impl::AssembleWall(const fe::NodalAssemblyView& assembly) {
    wall->trial_metrics=wall->accepted_metrics;
    const auto report=wall->contributor.AssembleAccepted(assembly,&wall->base);
    if(!report) return Convert(report);
    const auto& d=wall->base;
    if(!Identity(d,config,*wall->contributor.setup(),accepted.stamp,assembly.attempt)||
       d.phase!=wall_contact::NodalWallDevicePhase::AcceptedBase||d.time!=accepted.stamp.time||
       d.velocity_phase!=accepted.stamp.velocity_phase||d.velocity_time!=accepted.stamp.velocity_time||d.kick_dt!=0)
        return Failure(Status::ComponentFailure,"Contact base association differs from the actual owner assembly");
    return Success();
}
Report SourcePartElasticCase::Impl::EvaluateWall() {
    const auto report=wall->contributor.EvaluateCandidate(prepared,&wall->trial);
    if(!report) return Convert(report);
    const auto& d=wall->trial.diagnostics;
    if(!Identity(d,config,*wall->contributor.setup(),accepted.stamp,prepared.attempt)||
       d.phase!=wall_contact::NodalWallDevicePhase::PreparedCandidate||d.time!=prepared.proposed_time||
       d.velocity_phase!=prepared.velocity_phase||d.velocity_time!=prepared.velocity_time||d.kick_dt!=prepared.kick_dt)
        return Failure(Status::ComponentFailure,"Contact candidate association differs from the actual prepared owner");
    return Success();
}
void SourcePartElasticCase::Impl::CommitWall() noexcept {
    if(!wall) return;
    // Fixed-size, non-failing copies only after the sole owner/history commit.
    // Preserve PreparedCandidate provenance beside the new accepted owner stamp.
    wall->accepted=wall->trial;
    wall->accepted_metrics=wall->trial_metrics;
    wall->has_accepted_contact=true;
}
void SourcePartElasticCase::Impl::DiscardWall() noexcept {
    if(wall) wall->contributor.DiscardTrial();
}
const source_part_wall::SourcePartWallSetup* SourcePartElasticCase::wall_setup() const noexcept {
    return impl_->wall?impl_->wall->contributor.setup():nullptr;
}
const wall_contact::NodalWallDeviceResults* SourcePartElasticCase::accepted_contact() const noexcept {
    return impl_->wall&&impl_->wall->has_accepted_contact?&impl_->wall->accepted:nullptr;
}
const WallMetrics* SourcePartElasticCase::wall_metrics() const noexcept {
    return impl_->wall?&impl_->wall->accepted_metrics:nullptr;
}
} // namespace crash::cases::source_part_elastic
