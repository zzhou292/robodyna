#include "../loaded/Operations.h"
#include "lib_utest/qualification/physical_mesh_wall/Fixture.h"
namespace crash::cases::vehicle_wall::test {
namespace fixture=physical_wall_test;
namespace p=physical_publication_test;
namespace fe=tl::fea;
namespace c=tlfea::contact;
namespace {
bool Configure(c::NodalWallMappedContact& contact,p::Rig& rig) {
    return p::Good(rig.publication.ConfigurePhysicalScratchParticipation(
        rig.owner,rig.fixture.physical,rig.Participants(),rig.fixture.Identity(),
        {contact.roster_entry(),{}}));
}
}
TEST(VehicleLoadedWallCuda, SameCommonAttemptTransfersLoadsAndPublishesOnlyAfterCapture) {
    p::Rig rig(true);
    ASSERT_TRUE(rig.Initialize());
    fixture::Geometry geometry(rig.fixture);
    c::NodalWallMappedContact contact;
    ASSERT_TRUE(fixture::Good(contact.Initialize(fixture::Config(rig),geometry.Wall(),geometry.weights,
        fixture::Source(rig),rig.owner,geometry.Motion())));
    const auto entry=contact.roster_entry();
    fe::ShellPhysicalScratchParticipationForecast participation;
    ASSERT_TRUE(p::Good(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
        {entry,{}},{},participation)));
    EXPECT_EQ(participation.total_host_bytes,
        participation.publication_host_bytes+sizeof(fe::ShellPhysicalScratchParticipation));
    auto short_participation=fe::ShellPhysicalScratchParticipationLimits{
        participation.total_host_bytes-1};
    EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
        {entry,{}},short_participation,participation).status,
        fe::ShellPublicationStatus::ResourceLimit);
    ASSERT_TRUE(Configure(contact,rig));
    const auto allocations=contact.allocations();
    p::Snapshot initial,after;
    ASSERT_TRUE(rig.Read(initial));
    vehicle_dynamics::WallObservation discarded;
    for (unsigned attempt=0;attempt<3;++attempt) {
        fe::NodalTrialToken token;
        fe::NodalAssemblyView assembly;
        fe::NodalPreparedView prepared;
        fe::ShellPhysicalDiagnostics materials,common;
        c::NodalWallMappedTransactionReceipt receipt;
        vehicle_dynamics::WallObservation observation;
        ASSERT_TRUE(rig.Begin(token,assembly));
        ASSERT_NO_THROW(loaded::Assemble(contact,rig.owner,token,assembly,observation));
        EXPECT_TRUE(observation.enabled);
        EXPECT_FALSE(observation.prepared.valid);
        EXPECT_GT(observation.accepted.contact.resultant.value,0);
        // Reuses the independent owning fixture's declared +10 N load and
        // actual CIN witness transfer. Full app screening is tested separately.
        ASSERT_TRUE(rig.Advance(token,assembly,prepared));
        ASSERT_TRUE(rig.Evaluate(token,prepared,materials));
        ASSERT_TRUE(p::Good(rig.publication.PreparePhysical(rig.owner,token,
            {&materials.qeph,&materials.t3,&materials.qbat,&materials.type25,&materials.type13,&materials.solids},&common)));
        auto wrong_observation=observation;
        ++wrong_observation.accepted.contact.attempt;
        EXPECT_THROW(loaded::Evaluate(contact,rig.owner,token,prepared,common,wrong_observation,receipt),
            std::runtime_error);
        EXPECT_FALSE(receipt.valid());
        EXPECT_FALSE(wrong_observation.prepared.valid);
        ASSERT_NO_THROW(loaded::Evaluate(contact,rig.owner,token,prepared,common,observation,receipt));
        ASSERT_TRUE(receipt.valid());
        EXPECT_TRUE(observation.prepared.valid);
        EXPECT_TRUE(observation.prepared.prepared_activity_available);
        EXPECT_EQ(observation.prepared.accepted_active_parents,observation.accepted.accepted_active_parents);
        EXPECT_EQ(observation.prepared.contact.phase,c::NodalWallDevicePhase::PreparedCandidate);
        EXPECT_EQ(observation.prepared.contact.attempt,assembly.attempt);
        EXPECT_EQ(observation.prepared.contact.kick_dt,
            initial.stamp.fixed_dt*(rig.owner.accepted().epoch ? 1.0 : .5));
        EXPECT_GE(observation.prepared.contact.maximum_penetration,0);
        EXPECT_GE(observation.prepared.removed_potential.lower,0);
        fixture::Results rows(geometry);
        ASSERT_TRUE(fixture::Good(contact.CopyResults(observation.prepared,rows.View())));
        if (attempt==0) {
            discarded=observation;
            EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(
                rig.owner,token,{}).status,fe::ShellPublicationStatus::ParticipationFailure);
            contact.DiscardTrial();
            ASSERT_TRUE(rig.Read(after));
            p::Exact(initial,after);
            EXPECT_EQ(contact.CopyResults(discarded.prepared,rows.View()).status,c::NodalWallDeviceStatus::StaleAttempt);
        } else {
            if (attempt==1) {
                EXPECT_EQ(observation.prepared.contact.resultant.value,discarded.prepared.contact.resultant.value);
                EXPECT_EQ(observation.prepared.contact.potential.value,discarded.prepared.contact.potential.value);
                EXPECT_EQ(observation.prepared.contact.drift_work,discarded.prepared.contact.drift_work);
            }
            ASSERT_TRUE(p::Good(rig.publication.SealPhysicalScratchParticipation(
                rig.owner,token,receipt.scratch_receipts())));
            ASSERT_TRUE(p::Good(rig.publication.CommitPhysical(rig.owner,token,common,
                {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,p::Qualification,true})));
            EXPECT_EQ(rig.owner.accepted().epoch,attempt);
            EXPECT_EQ(contact.allocations().device_bytes,allocations.device_bytes);
            EXPECT_EQ(contact.allocations().device_allocations,allocations.device_allocations);
        }
    }
}
TEST(VehicleLoadedWallCuda, LateInvalidMaterialReceiptKeepsObservationAndAllAcceptedHistoriesThenRetry) {
    p::Rig rig(true);
    ASSERT_TRUE(rig.Initialize());
    fixture::Geometry geometry(rig.fixture);
    c::NodalWallMappedContact contact;
    ASSERT_TRUE(fixture::Good(contact.Initialize(fixture::Config(rig),geometry.Wall(),geometry.weights,
        fixture::Source(rig),rig.owner,geometry.Motion())));
    ASSERT_TRUE(Configure(contact,rig));
    p::Snapshot initial,after;
    ASSERT_TRUE(rig.Read(initial));
    for (unsigned attempt=0;attempt<2;++attempt) {
        fe::NodalTrialToken token;
        fe::NodalAssemblyView assembly;
        fe::NodalPreparedView prepared;
        fe::ShellPhysicalDiagnostics materials,common;
        c::NodalWallMappedTransactionReceipt receipt;
        vehicle_dynamics::WallObservation observation;
        ASSERT_TRUE(rig.Begin(token,assembly));
        ASSERT_NO_THROW(loaded::Assemble(contact,rig.owner,token,assembly,observation));
        ASSERT_TRUE(rig.Advance(token,assembly,prepared));
        ASSERT_TRUE(rig.Evaluate(token,prepared,materials));
        ASSERT_TRUE(p::Good(rig.publication.PreparePhysical(rig.owner,token,
            {&materials.qeph,&materials.t3,&materials.qbat,&materials.type25,&materials.type13,&materials.solids},&common)));
        if (!attempt) {
            const auto base=observation.accepted.contact.resultant.value;
            common.valid=false;
            EXPECT_THROW(loaded::Evaluate(contact,rig.owner,token,prepared,common,observation,receipt),
                std::runtime_error);
            EXPECT_EQ(observation.accepted.contact.resultant.value,base);
            EXPECT_FALSE(observation.prepared.valid);
            EXPECT_FALSE(receipt.valid());
            contact.DiscardTrial();
            rig.owner.Discard();
            rig.publication.DiscardTrial();
            ASSERT_TRUE(rig.Read(after));
            p::Exact(initial,after);
        } else {
            ASSERT_NO_THROW(loaded::Evaluate(contact,rig.owner,token,prepared,common,observation,receipt));
            ASSERT_TRUE(receipt.valid());
            ASSERT_TRUE(p::Good(rig.publication.SealPhysicalScratchParticipation(
                rig.owner,token,receipt.scratch_receipts())));
            ASSERT_TRUE(p::Good(rig.publication.CommitPhysical(rig.owner,token,common,
                {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,p::Qualification,true})));
            EXPECT_EQ(rig.owner.accepted().epoch,1);
        }
    }
}
} // namespace crash::cases::vehicle_wall::test
