#include "AcceptedWall.h"
#include "case/vehicle_dynamics/WallObservation.h"
namespace crash::output::physical_run::detail {
void CheckWallObservation(const cases::vehicle_dynamics::WallObservation& w,const tl::fea::NodalStamp& base,
        const tl::fea::NodalStamp& accepted,const records::Identity& id,const Values& v) {
    Require(w.enabled,"Loaded wall observation is unavailable");
    using Phase=tlfea::contact::NodalWallDevicePhase;
    const auto check=[&](const tlfea::contact::NodalWallMappedDiagnostics& d,Phase phase) {
        const auto& c=d.contact;
        Require(d.valid && c.valid && c.phase==phase && c.owner_id==v.owner &&
            c.configuration_id==id.configuration && c.qualification_id==id.qualification &&
            c.wall_binding_id && c.wall_binding_id==w.accepted.contact.wall_binding_id && c.attempt==v.stamp.attempt &&
            c.base_epoch==v.stamp.base_epoch && Bits(c.base_time)==Bits(v.stamp.base_time) &&
            Bits(c.base_velocity_time)==Bits(base.velocity_time) && c.scheme==base.temporal_scheme,
            "Committed wall observation source/force phase differs");
    };
    check(w.accepted,Phase::AcceptedBase);
    check(w.prepared,Phase::PreparedCandidate);
    const auto& a=w.accepted.contact;
    const auto& p=w.prepared.contact;
    Require(!w.accepted.prepared_activity_available && w.prepared.prepared_activity_available &&
        Bits(a.time)==Bits(v.stamp.base_time) && Bits(a.velocity_time)==Bits(base.velocity_time) &&
        Bits(a.kick_dt)==Bits(0.) && Bits(p.time)==Bits(v.stamp.time) &&
        Bits(p.velocity_time)==Bits(v.stamp.velocity_time) && Bits(p.kick_dt)==Bits(v.stamp.kick_dt) &&
        a.velocity_phase==base.velocity_phase && p.velocity_phase==accepted.velocity_phase &&
        a.node_count==p.node_count && a.parent_count==p.parent_count && a.node_count && a.parent_count &&
        w.accepted.accepted_active_parents==w.prepared.accepted_active_parents,
        "Committed wall interval/activity/shape differs");
}
} // namespace crash::output::physical_run::detail
