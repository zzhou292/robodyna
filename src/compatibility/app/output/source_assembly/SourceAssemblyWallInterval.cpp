#include "SourceAssemblyWallFields.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <algorithm>
#include <cmath>
#include <iterator>

namespace crash::output::assembly::wall_fields {
interval::Values IntervalValues(const tl::fea::NodalStamp& base,const dynamics::Diagnostics& d,dynamics::ContactView view) {
    const auto& s=d.stamp;Require(d.has_interval&&view.diagnostics&&view.nodes&&view.parents&&view.wall_face&&
        base.owner_id&&base.owner_id==s.owner_id&&base.node_count==s.node_count&&base.has_rotations==s.has_rotations&&
        base.fixed_dt==s.fixed_dt&&base.temporal_scheme==s.temporal_scheme&&
        tl::fea::SameRigidGroupInfo(base.rigid_groups,s.rigid_groups)&&base.epoch==s.reaction_base_epoch&&
        base.time==s.reaction_time&&s.epoch==base.epoch+1&&s.time==base.time+base.fixed_dt,
        "Assembly ledger row is not the next committed interval");
    const auto& c=*view.diagnostics;const auto& q=d.shells.qeph;const auto& t=d.shells.t3;
    Require(c.valid&&c.phase==tlfea::contact::NodalWallDevicePhase::PreparedCandidate&&c.owner_id==s.owner_id&&
        c.base_epoch==base.epoch&&c.base_time==base.time&&c.base_velocity_time==base.velocity_time&&
        c.time==s.time&&c.velocity_time==s.velocity_time&&c.kick_dt==s.reaction_kick_dt&&c.attempt&&
        c.attempt==q.attempt&&c.attempt==t.attempt&&q.phase==tl::fea::qeph::BatchPhase::Accepted&&
        t.phase==tl::fea::t3::BatchPhase::Accepted&&q.epoch==s.epoch&&t.epoch==s.epoch&&
        q.has_completed_interval&&t.has_completed_interval,"Assembly ledger contact/material publication mismatch");
    const auto same_family=[&](const auto& family) {
        return family.valid&&family.owner_id==s.owner_id&&family.base_epoch==base.epoch&&
            family.base_time==base.time&&family.base_velocity_time==base.velocity_time&&
            family.time==s.time&&family.velocity_time==s.velocity_time&&family.kick_dt==s.reaction_kick_dt&&
            family.configuration_id==c.configuration_id&&family.qualification_id==c.qualification_id;
    };
    Require(d.shells.valid&&same_family(q)&&same_family(t),
        "Assembly interval family/common/contact stamp mismatch");
    const auto& m=d.motion;
    const double values[]{s.velocity_time,s.reaction_kick_dt,m.after.native_total,m.after.effective_total,
        d.native_internal_work,d.cumulative_plastic_work,m.native_delta,m.effective_delta,m.replacement_delta,
        m.applied.total,m.reaction.total,m.native_residual,m.effective_residual,m.roundoff_budget,
        c.resultant.value,c.resultant.error,c.potential.value,c.potential.error,c.kick_work,c.drift_work,
        c.work_uncertainty,c.quadratic_work_upper,c.conservative_defect,c.wall_kick_impulse,c.wall_kick_impulse_error,
        c.maximum_penetration,double(d.active_contact_nodes),d.maximum_plastic_strain,double(d.yielded_points),double(d.yielded_parents),
        d.maximum_rotation,d.maximum_area_ratio,d.maximum_thickness_ratio};
    static_assert(sizeof(values)/sizeof(double)+6==WallIntervalColumns);
    interval::Values result;
    result.integers={base.owner_id,base.epoch,c.attempt,s.epoch};
    result.reals[interval::BaseTime]=base.time;
    result.reals[interval::Time]=s.time;
    std::copy(std::begin(values),std::end(values),result.reals.begin()+2);
    interval::CheckFinite(result);
    return result;
}
std::string IntervalRow(const tl::fea::NodalStamp& base,const dynamics::Diagnostics& d,dynamics::ContactView view) {
    return interval::CsvRow(IntervalValues(base,d,view));
}
} // namespace crash::output::assembly::wall_fields
