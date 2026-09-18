#include "CandidateFailureFixture.h"
#include "CandidateFailureValues.h"
#include "output/full_shell/tests/TestSupport.h"

#include <cstdlib>
#include <iostream>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {

TEST(CandidateFailureFixtureValues, RejectsUnboundedProofAndIncompleteExport) {
    contact::SelfContactTransactionLimits limits;
    CandidateFailureFixture fixture(limits);
    EXPECT_FALSE(fixture.seen());
    EXPECT_FALSE(fixture.complete());
    EXPECT_THROW(fixture.pair(), std::runtime_error);
    EXPECT_THROW(fixture.Export("unused-incomplete-failure-capture"), std::runtime_error);
    limits.crossing.max_work_per_pair = (1u << 20) + 1;
    EXPECT_THROW(CandidateFailureFixture{limits}, std::runtime_error);
}

TEST(CandidateFailureFixtureValues, CallbackRetainsFirstReportWithoutThrowingOrBorrowingMessage) {
    CandidateFailureFixture fixture(contact::SelfContactTransactionLimits{});
    const auto observer = fixture.observer();
    sct::CandidateFailureCapture invalid;
    char message[] = "native rejection retained";
    invalid.report.status = contact::SelfContactTransactionStatus::UnresolvedCandidate;
    invalid.report.message = message;
    observer.capture(observer.context, invalid);
    message[0] = 'X';
    EXPECT_TRUE(fixture.seen());
    EXPECT_FALSE(fixture.complete());
    EXPECT_STREQ(fixture.report().message, "native rejection retained");
    EXPECT_NE(fixture.error()[0], '\0');
    invalid.report.status = contact::SelfContactTransactionStatus::InvalidInput;
    observer.capture(observer.context, invalid);
    EXPECT_EQ(fixture.report().status, contact::SelfContactTransactionStatus::UnresolvedCandidate);
}

TEST(CandidateFailureFixtureValues, HostReplayRequiresExplicitManifestPin) {
    EXPECT_THROW(ReplayCandidateFailure("not-opened", ""), std::runtime_error);
}

TEST(CandidateFailureFixtureValues, UnreportedBaselineIsNotPresentedAsCertifiedSeparation) {
    // Empty geometry deliberately has no physical claim. This exercises only
    // frozen baseline provenance through the existing evaluator/projector.
    nonlinear_fixture::Pair pair;
    const auto result = failure_detail::Evaluate(pair, 1, 1, 0, nullptr, 0);
    EXPECT_EQ(result.baseline_status, pair.baseline_status);
    EXPECT_EQ(result.baseline_work, pair.baseline_work);
    EXPECT_EQ(result.baseline_depth, pair.baseline_depth);
    const auto document = prepared_replay::PairDocument(result);
    EXPECT_STREQ(document["baseline_status"].GetString(), "invalid_input");
    EXPECT_STREQ(document["baseline_depth_scope"].GetString(),
                 "unreported; not a measured production subdivision depth");

    // Corrected labels can differ from a historical artifact while the exact
    // numerical ledger/policy results remain identical.
    auto legacy = result;
    legacy.baseline_status = sct::NonlinearSeparationStatus::CertifiedSeparated;
    EXPECT_TRUE(failure_detail::Equivalent(legacy, result));
    EXPECT_FALSE(prepared_replay::PairDocument(legacy) == document);
}

TEST(CandidateFailureFixtureReplay, CallerPinnedCapturedPairReportsExactGeometryAndPolicy) {
    const auto* path = std::getenv("ROBO_SELF_CONTACT_FAILURE_MANIFEST");
    const auto* sha = std::getenv("ROBO_SELF_CONTACT_FAILURE_SHA256");
    ASSERT_TRUE(path && *path && sha && *sha)
        << "Set ROBO_SELF_CONTACT_FAILURE_MANIFEST and ROBO_SELF_CONTACT_FAILURE_SHA256";
    const auto report = ReplayCandidateFailure(path, sha);
    EXPECT_FALSE(report["physics_accepted"].GetBool());
    std::cout << failure_detail::JsonBytes(report) << std::endl;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
