#include "Support.h"
#include "../AcceptedWall.h"
#include "case/vehicle_dynamics/WallObservation.h"
namespace crash::output::physical_run::test {
TEST(PhysicalRunWallPhase, ActualAcceptedAndPreparedPhasesMustShareOwnerAttemptAndActivity) {
    const auto c=Context();const auto v=Row(c,1);
    tl::fea::NodalStamp base,accepted;
    base.owner_id=v.owner;base.temporal_scheme=tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
    accepted=base;accepted.velocity_phase=tl::fea::NodalVelocityPhase::PreviousMidpoint;
    cases::vehicle_dynamics::WallObservation wall;
    wall.enabled=true;
    auto& a=wall.accepted;auto& p=wall.prepared;
    a.valid=true;a.contact.valid=true;
    a.contact.owner_id=v.owner;a.contact.configuration_id=c.identity().configuration;
    a.contact.qualification_id=c.identity().qualification;a.contact.wall_binding_id=91;
    a.contact.attempt=v.stamp.attempt;a.contact.base_epoch=v.stamp.base_epoch;
    a.contact.scheme=base.temporal_scheme;a.contact.velocity_phase=base.velocity_phase;
    a.contact.time=a.contact.base_time=v.stamp.base_time;
    a.contact.phase=tlfea::contact::NodalWallDevicePhase::AcceptedBase;
    a.contact.node_count=7;a.contact.parent_count=4;a.accepted_active_parents=4;
    p=a;p.prepared_activity_available=true;p.proposed_active_parents=3;
    p.contact.phase=tlfea::contact::NodalWallDevicePhase::PreparedCandidate;
    p.contact.time=v.stamp.time;p.contact.velocity_time=v.stamp.velocity_time;
    p.contact.kick_dt=v.stamp.kick_dt;p.contact.velocity_phase=accepted.velocity_phase;
    EXPECT_NO_THROW(detail::CheckWallObservation(wall,base,accepted,c.identity(),v));
    for(unsigned i=0;i<10;++i) {
        auto bad=wall;
        switch(i) {
            case 0:bad.enabled=false;break;
            case 1:bad.prepared.contact.attempt++;break;
            case 2:bad.prepared.contact.wall_binding_id++;break;
            case 3:bad.accepted.contact.owner_id++;break;
            case 4:bad.accepted.contact.time=v.stamp.time;break;
            case 5:bad.prepared.contact.kick_dt*=2;break;
            case 6:bad.prepared.contact.phase=tlfea::contact::NodalWallDevicePhase::AcceptedBase;break;
            case 7:bad.prepared.accepted_active_parents--;break;
            case 8:bad.prepared.contact.node_count++;break;
            case 9:bad.prepared.prepared_activity_available=false;break;
        }
        EXPECT_THROW(detail::CheckWallObservation(bad,base,accepted,c.identity(),v),std::exception);
        EXPECT_NO_THROW(detail::CheckWallObservation(wall,base,accepted,c.identity(),v));
    }
}
} // namespace crash::output::physical_run::test
