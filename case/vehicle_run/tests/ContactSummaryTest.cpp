#include "../ContactSummary.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::cases::vehicle_run::test {
namespace {
vehicle_dynamics::StepObservation Observation(std::uint64_t epoch) {
    vehicle_dynamics::StepObservation result;
    result.wall.enabled=true;
    result.proposed_time=epoch*.001;
    result.wall.accepted.valid=true;
    result.wall.prepared.valid=true;
    result.wall.prepared.prepared_activity_available=true;
    result.wall.prepared.accepted_active_parents=10;
    result.wall.prepared.proposed_active_parents=9;
    result.wall.prepared.removed_potential.value=.25;
    auto& a=result.wall.accepted.contact;
    auto& b=result.wall.prepared.contact;
    for(auto* row:{&a,&b}) {
        row->valid=true;
        row->owner_id=7;
        row->base_epoch=epoch-1;
        row->attempt=epoch+4;
    }
    a.phase=tlfea::contact::NodalWallDevicePhase::AcceptedBase;
    b.phase=tlfea::contact::NodalWallDevicePhase::PreparedCandidate;
    a.time=(epoch-1)*.001;
    b.time=result.proposed_time;
    a.resultant.value=10;
    b.resultant.value=8;
    a.maximum_penetration=.001;
    b.maximum_penetration=.002;
    a.potential.value=2;
    b.potential.value=1;
    b.drift_work=-.5;
    return result;
}
tl::fea::NodalStamp Stamp(std::uint64_t epoch) {
    tl::fea::NodalStamp result;
    result.owner_id=7;
    result.epoch=epoch;
    result.time=epoch*.001;
    return result;
}
}
TEST(VehicleRunContact, ReportsBothActualEndpointsSignedWorkAndSeparateRemoval) {
    ContactTotals totals;
    ObserveAcceptedContact(totals,Observation(1),Stamp(1));
    EXPECT_TRUE(totals.available);
    EXPECT_EQ(totals.intervals,1u);
    EXPECT_EQ(totals.peak_observed_force_n,10);
    EXPECT_EQ(totals.peak_observed_penetration_m,.002);
    EXPECT_EQ(totals.peak_observed_potential_j,2);
    EXPECT_EQ(totals.last_same_mask_potential_j,1);
    EXPECT_EQ(totals.last_removed_potential_j,.25);
    EXPECT_EQ(totals.reported_drift_work_sum_j,-.5);
    EXPECT_EQ(totals.accepted_active_parents,10u);
    EXPECT_EQ(totals.proposed_active_parents,9u);
    auto next=Observation(2);
    next.wall.prepared.contact.resultant.value=12;
    next.wall.prepared.contact.drift_work=.125;
    ObserveAcceptedContact(totals,next,Stamp(2));
    EXPECT_EQ(totals.peak_observed_force_n,12);
    EXPECT_EQ(totals.reported_drift_work_sum_j,-.375);
}
TEST(VehicleRunContact, StalePhaseLateBadRemovalAndWorkOverflowPreservePriorSummary) {
    ContactTotals totals;
    ObserveAcceptedContact(totals,Observation(1),Stamp(1));
    for(unsigned bad=0;bad<4;++bad) {
        auto next=Observation(2);
        if(bad==0) next.wall.prepared.contact.phase=tlfea::contact::NodalWallDevicePhase::AcceptedBase;
        if(bad==1) next.wall.prepared.removed_potential.value=-1;
        if(bad==2) next.wall.prepared.contact.attempt++;
        if(bad==3) next.wall.prepared.contact.drift_work=std::numeric_limits<double>::infinity();
        EXPECT_THROW(ObserveAcceptedContact(totals,next,Stamp(2)),std::invalid_argument);
        EXPECT_EQ(totals.intervals,1u);
        EXPECT_EQ(totals.reported_drift_work_sum_j,-.5);
        EXPECT_EQ(totals.last_removed_potential_j,.25);
    }
    EXPECT_THROW(ObserveAcceptedContact(totals,Observation(1),Stamp(1)),std::invalid_argument);
    ObserveAcceptedContact(totals,Observation(2),Stamp(2));
    EXPECT_EQ(totals.intervals,2u);
    totals.reported_drift_work_sum_j=std::numeric_limits<double>::max();
    auto overflow=Observation(3);
    overflow.wall.prepared.contact.drift_work=std::numeric_limits<double>::max();
    EXPECT_THROW(ObserveAcceptedContact(totals,overflow,Stamp(3)),std::invalid_argument);
    EXPECT_EQ(totals.intervals,2u);
    EXPECT_EQ(totals.reported_drift_work_sum_j,std::numeric_limits<double>::max());
}
} // namespace crash::cases::vehicle_run::test
