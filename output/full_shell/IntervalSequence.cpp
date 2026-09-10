#include "IntervalSequence.h"
#include <algorithm>
#include <cmath>

namespace crash::output::full_shell {
void CheckIntervalContext(const IntervalContext& c) {
    CheckIdentity(c.identity);
    CheckStamp(c.fixed_dt,{});
    const auto& l=c.limits;
    Require(l.nodes&&l.nodes<=1048576&&l.parents&&l.parents<=1048576&&l.plastic_points<=4194304&&
        std::isfinite(l.maximum_penetration)&&l.maximum_penetration>0&&
        std::isfinite(l.maximum_rotation)&&l.maximum_rotation>0&&
        std::isfinite(l.maximum_area_ratio)&&l.maximum_area_ratio>0&&
        std::isfinite(l.maximum_thickness_ratio)&&l.maximum_thickness_ratio>0&&
        l.host_bytes&&l.host_bytes<=256*1024*1024,"Invalid interval source/envelope/host limits");
    const auto plan=interval::PlanChunks(c.planned_intervals,l.file_bytes);
    const auto active_rows=std::min(plan.rows_per_chunk,c.planned_intervals);
    Require(active_rows<=l.host_bytes/(4*interval::RowBytes),"Interval chunk staging exceeds host limit");
}

bool SameCursor(const IntervalCursor& a,const IntervalCursor& b) noexcept {
    return a.epoch==b.epoch&&a.attempt==b.attempt&&a.first_contact==b.first_contact&&
        a.last_contact==b.last_contact&&a.contact_intervals==b.contact_intervals&&
        Bits(a.time)==Bits(b.time)&&Bits(a.velocity_time)==Bits(b.velocity_time)&&Bits(a.base_time)==Bits(b.base_time)&&
        Bits(a.maximum_plastic)==Bits(b.maximum_plastic)&&Bits(a.plastic_work)==Bits(b.plastic_work)&&
        Bits(a.wall_potential)==Bits(b.wall_potential);
}

FrameStamp IntervalStamp(const interval::Values& v) {
    using namespace interval;
    return {v.integers[Epoch],v.integers[BaseEpoch],v.integers[Attempt],
        v.reals[Time],v.reals[BaseTime],v.reals[VelocityTime],v.reals[KickDt]};
}
void CheckCursor(const IntervalContext& c,const IntervalCursor& p) {
    Require(p.epoch<=c.planned_intervals&&std::isfinite(p.maximum_plastic)&&p.maximum_plastic>=0&&
        std::isfinite(p.plastic_work)&&p.plastic_work>=0&&std::isfinite(p.wall_potential)&&p.wall_potential>=0,
        "Invalid interval cursor values");
    if(!p.epoch) {
        Require(SameCursor(p,{}),"Interval ledger must start at the physical initial state");
        return;
    }
    CheckStamp(c.fixed_dt,{p.epoch,p.epoch-1,p.attempt,p.time,p.base_time,p.velocity_time,
        p.epoch==1?.5*c.fixed_dt:c.fixed_dt});
    Require(p.contact_intervals<=p.epoch&&
        (p.contact_intervals?(p.first_contact&&p.first_contact<=p.last_contact&&p.last_contact<=p.epoch&&
            p.contact_intervals<=p.last_contact-p.first_contact+1):(!p.first_contact&&!p.last_contact)),
        "Invalid interval contact cursor");
}

IntervalCursor AdvanceInterval(const IntervalContext& c,const IntervalCursor& before,const interval::Values& v) {
    using namespace interval;
    CheckIntervalContext(c);
    CheckCursor(c,before);
    CheckFinite(v);
    const auto stamp=IntervalStamp(v);
    CheckStamp(c.fixed_dt,stamp);
    Require(stamp.epoch&&stamp.epoch<=c.planned_intervals&&v.integers[Owner]==c.identity.owner&&
        stamp.base_epoch==before.epoch&&stamp.attempt>before.attempt&&
        Bits(stamp.base_time)==Bits(before.time),"Interval source/epoch/time/attempt sequence mismatch");
    if(!before.epoch)Require(SameCursor(before,{}),"Interval ledger must start at the physical initial state");
    const auto& r=v.reals;
    for(auto j:{NativeKinetic,EffectiveKinetic,CumulativePlasticWork,RoundoffBudget,WallResultant,WallResultantError,
        WallPotential,WallPotentialError,WallWorkUncertainty,WallQuadraticWorkUpper,WallKickImpulse,
        WallKickImpulseError,MaximumPenetration,ActiveNodes,MaximumPlasticStrain,YieldedPoints,YieldedParents,
        MaximumRotation,MaximumAreaRatio,MaximumThicknessRatio})
        Require(r[j]>=0,"Negative interval magnitude");
    const auto& l=c.limits;
    Require(r[MaximumPenetration]<=l.maximum_penetration&&r[MaximumRotation]<=l.maximum_rotation&&
        r[MaximumAreaRatio]<=l.maximum_area_ratio&&r[MaximumThicknessRatio]<=l.maximum_thickness_ratio&&
        r[ActiveNodes]<=l.nodes&&std::floor(r[ActiveNodes])==r[ActiveNodes]&&
        r[YieldedPoints]<=l.plastic_points&&std::floor(r[YieldedPoints])==r[YieldedPoints]&&
        r[YieldedParents]<=l.parents&&std::floor(r[YieldedParents])==r[YieldedParents]&&
        r[MaximumPlasticStrain]>=before.maximum_plastic&&r[CumulativePlasticWork]>=before.plastic_work,
        "Interval count, envelope or plastic history changed");
    Require(std::abs(r[NativeResidual])<=r[RoundoffBudget]&&std::abs(r[EffectiveResidual])<=r[RoundoffBudget],
        "Interval native recurrence/bookkeeping budget exceeded");
    IntervalCursor after=before;
    after.epoch=stamp.epoch;
    after.attempt=stamp.attempt;
    after.time=stamp.time;
    after.velocity_time=stamp.velocity_time;
    after.base_time=stamp.base_time;
    after.maximum_plastic=r[MaximumPlasticStrain];
    after.plastic_work=r[CumulativePlasticWork];
    after.wall_potential=r[WallPotential];
    if(r[ActiveNodes]>0) {
        if(!after.first_contact)after.first_contact=after.epoch;
        after.last_contact=after.epoch;
        ++after.contact_intervals;
    }
    return after;
}
} // namespace crash::output::full_shell
