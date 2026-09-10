#include "WallPenaltyCertification.h"
#include <cmath>

namespace crash::cases::wall_penalty {
namespace qb=contact::q4_bounds;
bool ValidPenaltyDesign(const PenaltyDesign& s) noexcept {
    return std::isfinite(s.area_floor)&&s.area_floor>0&&std::isfinite(s.design_penetration)&&s.design_penetration>0&&
        std::isfinite(s.penetration_cap)&&s.design_penetration<s.penetration_cap&&
        std::isfinite(s.kinetic_budget_factor)&&s.kinetic_budget_factor>1;
}
PenaltyReport CertifyAreaFloor(const contact::NodalWallWeights& weights,std::size_t count,
    double floor,AreaFloorCertificate* output) {
    using Code=PenaltyStatus;
    if(!output||!weights.prepared()||!count||weights.global_node_count()!=count||weights.node_count()!=count||
       !std::isfinite(floor)||floor<=0)return {Code::InvalidInput,"Area floor requires complete prepared unique nodes"};
    AreaFloorCertificate next;next.minimum_lower=HUGE_VAL;
    for(unsigned n=0;n<count;++n) {
        const auto& area=weights.node(n);
        if(area.node!=n||!contact::nodal_wall_detail::Certificate(area.area,true)||area.area.lower<floor)
            return {Code::CertificateFailure,"An assembled unique-node area does not certify the declared floor",n};
        if(area.area.lower<next.minimum_lower) {next.minimum_lower=area.area.lower;next.node=n;}
    }
    *output=next;return {Code::Ok,"Complete unique-node area floor certified"};
}
PenaltyReport CertifyPenalty(const contact::NodalWallWeights& weights,std::size_t count,
    const InitialKineticInput& energy,const PenaltyDesign& settings,PenaltyCertificate* output) {
    using Code=PenaltyStatus;
    if(!output||!weights.prepared()||!count||weights.global_node_count()!=count||weights.node_count()!=count||
       !ValidPenaltyDesign(settings)||
       (energy.metric!=InitialKineticMetric::NativePhysicalNodes&&energy.metric!=InitialKineticMetric::NativeNodesWithAggregateGroups)||
       !contact::nodal_wall_detail::Certificate(energy.enclosure,true))
        return {Code::InvalidInput,"Penalty certificate requires complete weights and an explicit initial physical-energy enclosure"};
    AreaFloorCertificate area;
    const auto checked=CertifyAreaFloor(weights,count,settings.area_floor,&area);if(!checked)return checked;
    PenaltyCertificate next;next.initial_kinetic=energy;
    next.minimum_nodal_area_lower=area.minimum_lower;next.minimum_area_node=area.node;
    if(energy.enclosure.lower<=0||
       !qb::MultiplyScalar(energy.enclosure.upper,settings.kinetic_budget_factor,true,&next.kinetic_budget_upper))
        return {Code::CertificateFailure,"Initial kinetic budget cannot be enclosed"};
    qb::Interval depth2,factor,quotient,proof;
    if(!qb::MultiplyPositive({settings.design_penetration,settings.design_penetration},
                            {settings.design_penetration,settings.design_penetration},&depth2)||
       !qb::Scale(depth2,settings.area_floor,&factor)||!qb::Scale(factor,.5,&factor)||factor.lower<=0||
       !qb::DividePositive({next.kinetic_budget_upper,next.kinetic_budget_upper},factor.lower,&quotient))
        return {Code::CertificateFailure,"Penetration design denominator cannot be enclosed"};
    next.stiffness_per_area=quotient.upper;
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
PenaltyReport CheckContactStep(double dt,double rate,double limit,double* upper) {
    using Code=PenaltyStatus;
    if(!upper||!std::isfinite(dt)||dt<=0||!std::isfinite(rate)||rate<=0||!std::isfinite(limit)||limit<=0||limit>.125)
        return {Code::InvalidInput,"Invalid contact-only stiffness guard input"};
    double root=0,next=0;
    if(!qb::Round(std::sqrt(rate),true,&root)||!qb::MultiplyScalar(dt,root,true,&next))
        return {Code::CertificateFailure,"Contact stiffness guard cannot be enclosed"};
    if(next>limit)return {Code::StepLimit,"Declared h*sqrt(contact stiffness rate) exceeds its frozen guard"};
    *upper=next;return {Code::Ok,"Contact-only stiffness guard certified"};
}
} // namespace crash::cases::wall_penalty
