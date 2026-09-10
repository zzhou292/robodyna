#include "SourceAssemblyWallFields.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace crash::output::assembly::wall_fields {
std::string IntervalRow(const tl::fea::NodalStamp& base,const dynamics::Diagnostics& d,dynamics::ContactView view) {
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
    const auto& m=d.motion;
    const double values[]{s.velocity_time,s.reaction_kick_dt,m.after.native_total,m.after.effective_total,
        d.native_internal_work,d.cumulative_plastic_work,m.native_delta,m.effective_delta,m.replacement_delta,
        m.applied.total,m.reaction.total,m.native_residual,m.effective_residual,m.roundoff_budget,
        c.resultant.value,c.resultant.error,c.potential.value,c.potential.error,c.kick_work,c.drift_work,
        c.work_uncertainty,c.quadratic_work_upper,c.conservative_defect,c.wall_kick_impulse,c.wall_kick_impulse_error,
        c.maximum_penetration,double(d.active_contact_nodes),d.maximum_plastic_strain,double(d.yielded_points),double(d.yielded_parents),
        d.maximum_rotation,d.maximum_area_ratio,d.maximum_thickness_ratio};
    static_assert(sizeof(values)/sizeof(double)+6==WallIntervalColumns);
    std::ostringstream row;row<<std::setprecision(17)<<base.owner_id<<','<<base.epoch<<','<<c.attempt<<','<<base.time<<','<<s.epoch<<','<<s.time;
    for(double x:values) {Require(std::isfinite(x),"Nonfinite accepted interval diagnostic");row<<','<<x;}row<<'\n';return row.str();
}
} // namespace crash::output::assembly::wall_fields
