#include "../Publication.h"
#include "case/vehicle_run/SelfContactDocument.h"
#include "lib_utest/qualification/self_contact_transaction/TransactionFixture.h"
#include "lib_src/collision/self_contact_transaction/CandidateFailureCapture.h"
#include "lib_src/collision/fixed_triangle_features/ExactPredicates.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>

namespace crash::cases::vehicle_run::diagnostics::test {
namespace {
namespace native=self_contact_transaction_cuda_test;
namespace p=physical_publication_test;
namespace c=tlfea::contact;
namespace fe=tl::fea;
namespace sct=c::self_contact_transaction;
namespace capture=vehicle_startup::shell_execution::self_contact_test;

struct Witness {
    sct::CandidateFailureObserver downstream;
    fe::NodalStamp accepted;
    fe::NodalPreparedView prepared;
    sct::QualificationPreparedCensusReceipt activity;
    unsigned calls=0;
    bool live=false, same_phase=false, nonlocal=false, caught=false;
    static void Observe(void* context,const sct::CandidateFailureCapture& value) noexcept {
        auto& witness=*static_cast<Witness*>(context);
        ++witness.calls;
        try {
            witness.activity=value.activity;
            witness.live=value.activity.valid() && value.motion.complete &&
                value.accepted_events.complete && value.accepted_assembly &&
                !value.accepted_assembly->valid();
            witness.same_phase=fe::trial_identity::SameStamp(witness.accepted,value.accepted) &&
                fe::trial_identity::SamePrepared(witness.prepared,value.prepared);
            if(value.facets.first<value.motion.facet_count && value.facets.second<value.motion.facet_count) {
                c::FixedTriangleIntersection intersection;
                bool intersects=false;
                witness.nonlocal=c::fixed_triangle_features::ClassifyPairIntersection(
                    value.motion.prepared_triangles[value.facets.first],
                    value.motion.prepared_triangles[value.facets.second],&intersection,&intersects)==
                    c::FixedTriangleDiscoveryStatus::Ok && intersects &&
                    c::RequiresIntersectionAdmission(intersection);
            }
            witness.downstream.capture(witness.downstream.context,value);
        } catch(...) { witness.caught=true; }
    }
    sct::CandidateFailureObserver observer() noexcept {return {this,sizeof(*this),Observe};}
};

TEST(VehicleFailureNative, RealEpochTwoRejectionExportsExactPhaseAfterRollback) {
    native::Fixture fixture(false,true);
    const auto load_node=fixture.rig.external_force_source_node;
    const auto load_force=fixture.rig.external_force_z_n;
    fixture.rig.external_force_source_node=0;
    const c::SelfContactTransactionLimits limits;
    ASSERT_TRUE(fixture.Initialize(limits));
    capture::CandidateFailureFixture evidence(limits);
    Witness witness;
    witness.downstream=evidence.observer();
    // The ordinary owner and common publication commit both intervals. The
    // observer is installed throughout; successful candidates never call it.
    for(unsigned interval=0;interval<2;++interval) {
        fe::NodalTrialToken token;
        fe::NodalAssemblyView assembly;
        ASSERT_TRUE(fixture.rig.Begin(token,assembly));
        c::SelfContactAcceptedAssemblyReceipt accepted;
        ASSERT_TRUE(native::Good(fixture.transaction.AssembleAccepted(
            fixture.rig.owner,token,assembly,&accepted)));
        fe::NodalPreparedView prepared;
        fe::ShellPhysicalDiagnostics common;
        ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));
        witness.accepted=fixture.rig.owner.accepted();witness.prepared=prepared;
        const auto observer=witness.observer();
        c::SelfContactTransactionReceipt receipt;
        ASSERT_TRUE(native::Good(sct::QualificationAccess::SealCandidateWithFailureObserver(
            fixture.transaction,fixture.rig.owner,token,common,prepared,accepted,&receipt,observer)));
        ASSERT_TRUE(fixture.Commit(token,prepared,common,receipt));
        ASSERT_EQ(fixture.rig.owner.accepted().epoch,interval+1u);
        EXPECT_EQ(witness.calls,0u);
        EXPECT_FALSE(evidence.seen());
    }
    p::Snapshot before,after;
    ASSERT_TRUE(fixture.rig.Read(before));
    ASSERT_EQ(before.stamp.epoch,2u);
    fixture.rig.external_force_source_node=load_node;
    fixture.rig.external_force_z_n=load_force;
    c::SelfContactTransactionReport baseline;
    fe::NodalPreparedView captured_prepared;
    for(unsigned attempt=0;attempt<2;++attempt) {
        fe::NodalTrialToken token;
        fe::NodalAssemblyView assembly;
        ASSERT_TRUE(fixture.rig.Begin(token,assembly));
        c::SelfContactAcceptedAssemblyReceipt accepted;
        ASSERT_TRUE(native::Good(fixture.transaction.AssembleAccepted(
            fixture.rig.owner,token,assembly,&accepted)));
        fe::NodalPreparedView prepared;
        fe::ShellPhysicalDiagnostics common;
        ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));
        witness.accepted=before.stamp;witness.prepared=prepared;
        const auto observer=witness.observer();
        c::SelfContactTransactionReceipt receipt;
        const auto report=attempt
            ? sct::QualificationAccess::SealCandidateWithFailureObserver(fixture.transaction,
                fixture.rig.owner,token,common,prepared,accepted,&receipt,observer)
            : fixture.transaction.SealCandidate(fixture.rig.owner,token,common,prepared,accepted,&receipt);
        ASSERT_EQ(report.status,c::SelfContactTransactionStatus::CandidateRejected)<<report.message;
        EXPECT_FALSE(receipt.valid());EXPECT_FALSE(accepted.valid());
        if(!attempt) {baseline=report;EXPECT_FALSE(evidence.seen());}
        else {
            captured_prepared=prepared;
            EXPECT_TRUE(vehicle_run::detail::SelfContactErrorDocument(
                vehicle_self_contact::SelfContactStageError(report,
                    vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal,512))==
                vehicle_run::detail::SelfContactErrorDocument(vehicle_self_contact::SelfContactStageError(
                    baseline,vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal,512)));
        }
        ASSERT_TRUE(fixture.rig.Read(after));p::Exact(before,after);
    }
    ASSERT_EQ(witness.calls,1u);
    EXPECT_TRUE(witness.live);EXPECT_TRUE(witness.same_phase);EXPECT_TRUE(witness.nonlocal);
    EXPECT_FALSE(witness.caught);EXPECT_FALSE(witness.activity.valid());
    ASSERT_TRUE(evidence.seen());ASSERT_TRUE(evidence.complete())<<evidence.error();
    ASSERT_TRUE(evidence.owners_equivalent());

    // Adapt actual coupon result values to the same post-run publication
    // checker used by the CLI. This is not a fabricated physical owner stamp
    // or a claim that a full vehicle controller/archive ran in this coupon.
    Result run;
    run.loop.kind=StopKind::PhysicsRejected;
    run.loop.progress.accepted={after.stamp.epoch,after.stamp.time};
    run.rejected_self_contact.emplace(baseline,
        vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal,512);
    output::full_shell::test::Directory temporary;
    const auto* requested=std::getenv("ROBO_LATER_EPOCH_FAILURE_OUTPUT");
    const auto destination=requested && *requested ? std::filesystem::path(requested)
        : temporary.path/"failure";
    const auto published=detail::PublishFailure(run,evidence,destination);
    ASSERT_EQ(published.status,FailureStatus::Captured)<<published.error;
    const auto bytes=output::ReadBounded(published.manifest,capture::FailureFixtureManifestCap);
    EXPECT_EQ(output::Sha256(bytes),published.sha256);
    const auto manifest=output::array_json::Parse(bytes,capture::FailureFixtureManifestCap);
    EXPECT_EQ(manifest["owner_id"].GetUint64(),after.stamp.owner_id);
    EXPECT_EQ(manifest["accepted_epoch"].GetUint64(),after.stamp.epoch);
    EXPECT_EQ(manifest["attempt"].GetUint64(),captured_prepared.attempt);
    EXPECT_EQ(manifest["duration_bits"].GetUint64(),
        output::Bits(captured_prepared.proposed_time-captured_prepared.base_time));
    EXPECT_EQ(manifest["kick_dt_bits"].GetUint64(),output::Bits(captured_prepared.kick_dt));
    EXPECT_TRUE(manifest["native_report"]==vehicle_run::detail::SelfContactErrorDocument(*run.rejected_self_contact));
    const auto frozen=capture::nonlinear_fixture::Read((destination/"pair.bin").string(),false);
    EXPECT_EQ(frozen.phase.accepted_epoch,after.stamp.epoch);
    EXPECT_EQ(frozen.phase.prepared_base_epoch,after.stamp.epoch);
    EXPECT_EQ(frozen.phase.accepted_time_bits,output::Bits(after.stamp.time));
    EXPECT_EQ(frozen.phase.prepared_time_bits,output::Bits(captured_prepared.proposed_time));
    EXPECT_EQ(frozen.phase.accepted_velocity_time_bits,output::Bits(after.stamp.velocity_time));
    EXPECT_EQ(frozen.phase.prepared_velocity_time_bits,output::Bits(captured_prepared.velocity_time));
    const auto replay=capture::ReplayCandidateFailure(published.manifest,published.sha256);
    EXPECT_TRUE(replay["matches_captured_compact_result"].GetBool());
    EXPECT_FALSE(replay["physics_accepted"].GetBool());
    EXPECT_THROW(evidence.Export(destination),std::exception);
    ASSERT_TRUE(fixture.rig.Read(after));p::Exact(before,after);
    RecordProperty("scope","small native owner after two real commits; terminal rejection/export only, not full vehicle acceptance");
    RecordProperty("accepted_epoch",std::to_string(after.stamp.epoch));
    RecordProperty("owner_id",std::to_string(after.stamp.owner_id));
    RecordProperty("failure_manifest",published.manifest.string());
    RecordProperty("failure_manifest_sha256",published.sha256);
}
} // namespace
} // namespace crash::cases::vehicle_run::diagnostics::test
