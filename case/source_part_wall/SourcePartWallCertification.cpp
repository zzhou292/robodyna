#include "SourcePartWallCertification.h"
#include "collision/Q4ContactBounds.h"
#include <cmath>

namespace crash::cases::source_part_wall {
namespace qb=contact::q4_bounds;
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
    SourcePartWallCertificate next;next.minimum_nodal_area_lower=HUGE_VAL;
    for(unsigned n=0;n<source::NodeCount;++n) {
        const auto& area=weights.node(n);
        if(area.node!=n||!contact::nodal_wall_detail::Certificate(area.area,true)||area.area.lower<settings.area_floor)
            return {Code::CertificateFailure,"An assembled unique-node area does not certify the declared floor",n};
        if(area.area.lower<next.minimum_nodal_area_lower) {next.minimum_nodal_area_lower=area.area.lower;next.minimum_area_node=n;}
    }
    qb::Interval speed2,energy;double nominal=0;
    const double speed=settings.initial_velocity[0];
    if(!qb::MultiplyPositive({speed,speed},{speed,speed},&speed2))
        return {Code::CertificateFailure,"Initial speed square cannot be enclosed"};
    for(unsigned n=0;n<source::NodeCount;++n) {
        const double mass=binding.nodes()[n].native.mass;qb::Interval term;
        if(!std::isfinite(mass)||mass<=0||!qb::Scale(speed2,mass,&term)||!qb::Scale(term,.5,&term)||!qb::Add(energy,term,&energy))
            return {Code::CertificateFailure,"Native initial kinetic reduction cannot be enclosed",n};
        nominal+=.5*mass*(speed*speed);
        if(!std::isfinite(nominal))return {Code::CertificateFailure,"Native initial kinetic value overflow",n};
    }
    if(!qb::Certify(nominal,energy,&next.native_initial_kinetic)||energy.lower<=0||
       !qb::MultiplyScalar(energy.upper,settings.kinetic_budget_factor,true,&next.kinetic_budget_upper))
        return {Code::CertificateFailure,"Initial kinetic budget cannot be enclosed"};
    qb::Interval depth2,factor,quotient,proof;
    if(!qb::MultiplyPositive({settings.design_penetration,settings.design_penetration},
                            {settings.design_penetration,settings.design_penetration},&depth2)||
       !qb::Scale(depth2,settings.area_floor,&factor)||!qb::Scale(factor,.5,&factor)||factor.lower<=0||
       !qb::DividePositive({next.kinetic_budget_upper,next.kinetic_budget_upper},factor.lower,&quotient))
        return {Code::CertificateFailure,"Penetration design denominator cannot be enclosed"};
    next.stiffness_per_area=quotient.upper;
    // Directed quotient plus a bounded upward ULP walk closes any remaining
    // lower-product rounding. This is certification of the same fixed energy
    // inequality, not a response-dependent change to physics or tolerance.
    for(unsigned step=0;step<=64;++step) {
        if(!qb::Scale(factor,next.stiffness_per_area,&proof))break;
        if(proof.lower>=next.kinetic_budget_upper) {
            next.design_potential_lower=proof.lower;next.kappa_upward_steps=step;*output=next;
            return {Code::Ok,"Individual-node area floor and penalty energy inequality certified"};
        }
        next.stiffness_per_area=std::nextafter(next.stiffness_per_area,HUGE_VAL);
        if(!std::isfinite(next.stiffness_per_area))break;
    }
    return {Code::CertificateFailure,"Upward penalty selection did not close its lower-bound proof"};
}
} // namespace detail
SourcePartWallReport CheckSourcePartContactStep(double dt,double rate,double limit,double* upper) {
    using Code=SourcePartWallStatus;
    if(!upper||!std::isfinite(dt)||dt<=0||!std::isfinite(rate)||rate<=0||!std::isfinite(limit)||limit<=0||limit>.125)
        return {Code::InvalidInput,"Invalid contact-only stiffness guard input"};
    double root=0,next=0;
    if(!qb::Round(std::sqrt(rate),true,&root)||!qb::MultiplyScalar(dt,root,true,&next))
        return {Code::CertificateFailure,"Contact stiffness guard cannot be enclosed"};
    if(next>limit)return {Code::StepLimit,"Declared h*sqrt(contact stiffness rate) exceeds its frozen guard"};
    *upper=next;return {Code::Ok,"Contact-only stiffness guard certified"};
}
} // namespace crash::cases::source_part_wall
