#include "SourcePartWallCertification.h"
#include "case/wall_penalty/WallPenaltyCertification.h"
#include "case/wall_penalty/UniformTranslationKinetic.h"
#include <cmath>

namespace crash::cases::source_part_wall {
namespace penalty=wall_penalty;
namespace {
SourcePartWallReport Convert(penalty::PenaltyReport report) {
    using S=penalty::PenaltyStatus;using D=SourcePartWallStatus;
    const auto status=report.status==S::Ok?D::Ok:report.status==S::InvalidInput?D::InvalidInput:
        report.status==S::StepLimit?D::StepLimit:D::CertificateFailure;
    return {status,report.message,report.node};
}
} // namespace
namespace detail {
bool ValidWallSettings(const SourcePartWallSettings& s) noexcept {
    for(double value:{s.initial_velocity[0],s.leading_gap,s.area_floor,s.design_penetration,s.penetration_cap,
        s.kinetic_budget_factor,s.motion_margin,s.exposed_clearance,s.parent_force_error,s.parent_energy_error,s.maximum_step_rate})
        if(!std::isfinite(value)||value<=0)return false;
    return s.initial_velocity[1]==0&&s.initial_velocity[2]==0&&s.kinetic_budget_factor>1&&
        s.design_penetration<s.penetration_cap&&s.exposed_clearance<s.motion_margin&&s.maximum_step_rate<=.125&&
        s.configuration_id&&s.qualification_id&&s.wall_binding_id;
}
SourcePartWallReport CertifyWallPenalty(const tl::fea::ShellBatchBinding& binding,const contact::NodalWallWeights& weights,
    const SourcePartWallSettings& settings,SourcePartWallCertificate* output) {
    using Code=SourcePartWallStatus;
    if(!output||!binding.prepared()||!weights.prepared()||!ValidWallSettings(settings)||
       binding.node_count()!=source::NodeCount||weights.node_count()!=source::NodeCount)
        return {Code::InvalidInput,"Penalty certificate requires the entire prepared original native union"};
    // Retain the legacy area-before-energy failure order as well as its exact
    // nominal native-node reduction; the source-neutral utility owns the math.
    penalty::AreaFloorCertificate area;
    const auto checked=penalty::CertifyAreaFloor(weights,source::NodeCount,settings.area_floor,&area);
    if(!checked)return Convert(checked);
    penalty::UniformTranslationKinetic energy;
    if(!penalty::BeginUniformTranslation(settings.initial_velocity[0],&energy))
        return {Code::CertificateFailure,"Initial speed square cannot be enclosed"};
    for(unsigned n=0;n<source::NodeCount;++n)
        if(!penalty::AddTranslationMass(binding.nodes()[n].native.mass,&energy))
            return {Code::CertificateFailure,"Native initial kinetic reduction cannot be enclosed",n};
    SourcePartWallCertificate next;
    if(!penalty::FinishUniformTranslation(energy,&next.native_initial_kinetic))
        return {Code::CertificateFailure,"Initial kinetic budget cannot be enclosed"};
    penalty::PenaltyCertificate certified;
    const auto report=penalty::CertifyPenalty(weights,source::NodeCount,
        {next.native_initial_kinetic,penalty::InitialKineticMetric::NativePhysicalNodes},
        {settings.area_floor,settings.design_penetration,settings.penetration_cap,settings.kinetic_budget_factor},&certified);
    if(!report)return Convert(report);
    next.kinetic_budget_upper=certified.kinetic_budget_upper;next.stiffness_per_area=certified.stiffness_per_area;
    next.design_potential_lower=certified.design_potential_lower;
    next.minimum_nodal_area_lower=certified.minimum_nodal_area_lower;next.minimum_area_node=certified.minimum_area_node;
    next.kappa_upward_steps=certified.kappa_upward_steps;*output=next;return Convert(report);
}
} // namespace detail
SourcePartWallReport CheckSourcePartContactStep(double dt,double rate,double limit,double* upper) {
    return Convert(penalty::CheckContactStep(dt,rate,limit,upper));
}
} // namespace crash::cases::source_part_wall
