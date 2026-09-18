#include "../Run.h"
#include "OriginalFixture.h"
#include "../ContactComposition.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateCapture.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateFailureFixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <iostream>

namespace crash::cases::vehicle_run::test {
namespace {
namespace capture = vehicle_startup::shell_execution::self_contact_test;

struct DiscardFailureCandidate {
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics;
    ~DiscardFailureCandidate() { dynamics.DiscardStep(); }
};

}  // namespace

TEST(VehicleRunWallSelfContactFailure, CaptureFirstNativeRejectionWithinTwoIntervals) {
    const auto* destination = std::getenv("ROBO_SELF_CONTACT_FAILURE_OUTPUT");
    ASSERT_TRUE(destination && *destination)
        << "Set an absent ROBO_SELF_CONTACT_FAILURE_OUTPUT directory";
    ASSERT_FALSE(std::filesystem::exists(destination));
    std::cout << std::unitbuf;
    const auto source = Source(PhysicalProfile::VehicleSupportsV5, nullptr,
                               ContactProfile::WallSelfContactV1);
    auto config = vehicle_wall::LoadedWallConfig();
    config.startup.reserved_step_s = 2e-7;
    config.timing.enabled = true;
    const auto composition = ContactComposition::Prepare(
        ContactProfile::WallSelfContactV1, source.self_contact);
    const auto forecast = composition.Preflight(source.setup, config, &source.joints);
    ASSERT_LE(forecast.peak_host_upper_bound + capture::FailureCaptureHostCap,
              20ull * 1000 * 1000 * 1000);
    capture::CandidateFailureFixture fixture(composition.runtime_limits().self_contact.transaction);
    const auto observer = fixture.observer();  // Descriptor is outside output context.
    auto dynamics = composition.CreateDynamics(source.setup, config, &source.joints);
    DiscardFailureCandidate cleanup{dynamics};
    for (unsigned interval = 0; interval < 2; ++interval) {
        SCOPED_TRACE(interval);
        const auto accepted = dynamics.accepted();
        const auto report = vehicle_self_contact::CandidateRigidCouponAccess::PrepareWithFailureObserver(
            dynamics, observer);
        if (report.status == tlfea::contact::SelfContactTransactionStatus::Ok) {
            dynamics.CommitStep();
            std::cout << "V5_FAILURE_CAPTURE committed_epoch=" << dynamics.accepted().epoch << std::endl;
            continue;
        }
        ASSERT_TRUE(fixture.seen()) << report.message;
        ASSERT_TRUE(fixture.complete()) << fixture.error();
        ASSERT_TRUE(fixture.owners_equivalent());
        ASSERT_EQ(fixture.report().status, report.status);
        ASSERT_FALSE(dynamics.has_prepared_step());
        ASSERT_TRUE(tl::fea::trial_identity::SameStamp(dynamics.accepted(), accepted));
        const auto sha = fixture.Export(destination);
        const auto manifest = std::filesystem::path(destination) / "failure.json";
        std::cout << "V5_FAILURE_CAPTURE native_rejected=1 accepted_epoch=" << accepted.epoch
                  << " manifest=" << manifest << " sha256=" << sha << std::endl;
        RecordProperty("scope", "diagnostic capture of native rejection; no physics acceptance");
        RecordProperty("failure_manifest", manifest.string());
        RecordProperty("failure_manifest_sha256", sha);
        RecordProperty("accepted_epoch", static_cast<int>(accepted.epoch));
        // Capture first, then qualify the currently observed production failure.
        // Changed behavior retains its evidence before this assertion reports it.
        EXPECT_EQ(report.status, tlfea::contact::SelfContactTransactionStatus::UnresolvedCandidate);
        EXPECT_EQ(accepted.epoch, 0u);
        EXPECT_EQ(fixture.pair().prepared[0].key.parent_eid, 2107699u);
        EXPECT_EQ(fixture.pair().prepared[0].key.local_facet, 1u);
        EXPECT_EQ(fixture.pair().prepared[1].key.parent_eid, 2320241u);
        EXPECT_EQ(fixture.pair().prepared[1].key.local_facet, 1u);
        return;
    }
    FAIL() << "No native rejection in bounded two-interval diagnostic; no failure artifact claimed";
}

}  // namespace crash::cases::vehicle_run::test
