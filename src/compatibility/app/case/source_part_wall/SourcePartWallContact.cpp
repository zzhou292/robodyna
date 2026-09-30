#include "SourcePartWallContact.h"
#include <new>

namespace crash::cases::source_part_wall {
namespace {
using Code=SourcePartWallStatus;
SourcePartWallReport Convert(const contact::NodalWallDeviceReport& report) noexcept {
    if(report.status==contact::NodalWallDeviceStatus::Ok)return {Code::Ok,report.message};
    const auto status=report.status==contact::NodalWallDeviceStatus::DeviceFailure?Code::DeviceFailure:
        report.status==contact::NodalWallDeviceStatus::ResourceLimit?Code::ResourceLimit:Code::ContributorFailure;
    return {status,report.message,report.node,report.parent};
}
} // namespace
struct SourcePartWallContact::Impl {
    SourcePartWallSetup setup;
    contact::NodalWallContactDevice contributor;
    // Fixed-capacity host scratch is allocated once with Impl. No per-step
    // allocations, retained device result aliases or renderer-owned state.
    contact::NodalWallDeviceResults results;
    double step_rate_upper=0;
};
SourcePartWallContact::SourcePartWallContact()=default;
SourcePartWallContact::~SourcePartWallContact()=default;
SourcePartWallReport SourcePartWallContact::Initialize(const source::SourcePartContactFixture& input,
    const tl::fea::ShellBatchBinding& binding,const tl::fea::NodalStamp& stamp,const double* inverse,
    double measured_initial_kinetic,const case_data::CanonicalWall& canonical,const std::string& bytes,
    const SourcePartWallSettings& settings) {
    if(impl_)return {Code::AlreadyInitialized,"Source wall contributor is immutable after initialization"};
    try {
        auto next=std::make_unique<Impl>();
        auto result=next->setup.Initialize(input,binding,stamp,inverse,measured_initial_kinetic,canonical,bytes,settings);
        if(!result)return result;
        const auto& setup=next->setup;const auto& certificate=*setup.certificate();
        contact::NodalWallDeviceConfig config;
        config.owner=*setup.owner_stamp();config.configuration_id=settings.configuration_id;
        config.qualification_id=settings.qualification_id;config.wall_binding_id=settings.wall_binding_id;
        config.law={setup.placed_wall()->geometry()->wall_x(),certificate.stiffness_per_area,
                    settings.penetration_cap,settings.parent_force_error,settings.parent_energy_error};
        config.exposed_clearance=settings.exposed_clearance;
        result=Convert(next->contributor.Initialize(config,setup.placed_wall()->view(),
            *setup.source_geometry()->weights(),{setup.initial_positions(),source::NodeCount,3,1},
            setup.inverse_mass(),setup.translation_fixed_bits(),certificate.coverage.physical));
        if(!result)return result;
        // Use the contributor's owning all-active assembled rate, including its
        // native inverse masses and rounded parent coefficients. A failed guard
        // destroys this unpublished participant and releases its one allocation.
        result=CheckSourcePartContactStep(stamp.fixed_dt,next->contributor.stiffness_rate_bound(),
                                         settings.maximum_step_rate,&next->step_rate_upper);
        if(!result)return result;
        impl_=std::move(next);return {Code::Ok,"Source finite-wall contributor and contact-only step guard prepared"};
    } catch(const std::bad_alloc&) {
        return {Code::ResourceLimit,"Bounded source wall contributor allocation failed"};
    }
}
SourcePartWallReport SourcePartWallContact::AssembleAccepted(const tl::fea::NodalAssemblyView& view,
    contact::NodalWallDiagnostics* output) {
    if(!impl_)return {Code::NotInitialized,"Source wall contributor is not initialized"};
    if(!output) {DiscardTrial();return {Code::InvalidInput,"Missing accepted-base contact diagnostics destination"};}
    contact::NodalWallDiagnostics next;
    const auto result=Convert(impl_->contributor.AssembleAccepted(view,&next));
    if(!result) {DiscardTrial();return result;}
    *output=next;return result;
}
SourcePartWallReport SourcePartWallContact::EvaluateCandidate(const tl::fea::NodalPreparedView& view,
    contact::NodalWallDeviceResults* output) {
    if(!impl_)return {Code::NotInitialized,"Source wall contributor is not initialized"};
    if(!output) {DiscardTrial();return {Code::InvalidInput,"Missing staged contact result destination"};}
    contact::NodalWallDiagnostics next;
    auto result=Convert(impl_->contributor.EvaluateCandidate(view,&next));
    if(result)result=Convert(impl_->contributor.CopyResults(next,&impl_->results));
    if(!result) {DiscardTrial();return result;}
    *output=impl_->results;return result;
}
void SourcePartWallContact::DiscardTrial() noexcept {if(impl_)impl_->contributor.DiscardTrial();}
bool SourcePartWallContact::initialized() const noexcept {return impl_!=nullptr;}
const SourcePartWallSetup* SourcePartWallContact::setup() const noexcept {return impl_?&impl_->setup:nullptr;}
tl::fea::NodalAllocationInfo SourcePartWallContact::allocations() const noexcept {
    return impl_?impl_->contributor.allocations():tl::fea::NodalAllocationInfo{};
}
double SourcePartWallContact::stiffness_rate_bound() const noexcept {return impl_?impl_->contributor.stiffness_rate_bound():0;}
double SourcePartWallContact::step_rate_upper() const noexcept {return impl_?impl_->step_rate_upper:0;}
} // namespace crash::cases::source_part_wall
