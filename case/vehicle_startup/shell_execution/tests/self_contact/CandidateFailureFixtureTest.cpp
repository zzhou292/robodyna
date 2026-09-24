#include "CandidateFailureFixture.h"
#include "CandidateFailureValues.h"
#include "FailureBaselineScope.h"
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

TEST(CandidateFailureFixtureValues, OptionalObservedBaselineKeepsAbsentAndCombinedResultsDistinct) {
    sct::CandidateFailureCapture input;
    input.nonlinear_baseline.status = sct::NonlinearSeparationStatus::DepthExhausted;
    input.nonlinear_baseline.work = 96;
    input.nonlinear_baseline.deepest = 20;
    input.nonlinear_baseline.depth_exhausted = true;
    input.nonlinear_baseline.proof_digest = 123456;
    auto absent = failure_detail::CapturedNonlinearBaseline(input);
    EXPECT_EQ(absent.status, sct::NonlinearSeparationStatus::InvalidInput);
    EXPECT_EQ(absent.work, 0u);
    EXPECT_EQ(absent.deepest, 0u);
    EXPECT_FALSE(absent.depth_exhausted);

    input.has_nonlinear_baseline = true;
    const auto present = failure_detail::CapturedNonlinearBaseline(input);
    EXPECT_EQ(present.status, input.nonlinear_baseline.status);
    EXPECT_EQ(present.work, input.nonlinear_baseline.work);
    EXPECT_EQ(present.deepest, input.nonlinear_baseline.deepest);
    EXPECT_EQ(present.depth_exhausted, input.nonlinear_baseline.depth_exhausted);
    EXPECT_EQ(present.proof_digest, input.nonlinear_baseline.proof_digest);

    // Metadata-only empty geometry: no physical or replay-success claim.
    nonlinear_fixture::Pair pair;
    pair.baseline_status = present.status;
    pair.baseline_work = present.work;
    pair.baseline_depth = present.deepest;
    const auto ordinary = failure_detail::Evaluate(pair, 1, 1, 0, nullptr, 0);
    const auto observed = failure_detail::Evaluate(pair, 1, 1, 0, nullptr, 0, true);
    const auto unreported_document = prepared_replay::PairDocument(ordinary);
    const auto observed_document = prepared_replay::PairDocument(observed);
    EXPECT_STREQ(unreported_document["baseline_depth_scope"].GetString(),
                 failure_detail::UnreportedBaselineDepthScope);
    EXPECT_STREQ(observed_document["baseline_depth_scope"].GetString(),
                 failure_detail::ObservedBaselineDepthScope);
    EXPECT_EQ(observed_document["baseline_work"].GetUint64(), 96u);
    EXPECT_EQ(observed_document["baseline_depth"].GetUint64(), 20u);
    EXPECT_STREQ(observed_document["baseline_status"].GetString(), "depth_exhausted");
    EXPECT_EQ(observed_document["ledger"], unreported_document["ledger"]);
    EXPECT_EQ(observed_document["policy"], unreported_document["policy"]);
    EXPECT_TRUE(failure_detail::Equivalent(ordinary, observed));
}

TEST(CandidateFailureFixtureValues, ExplicitMetadataVersionPreservesLegacyAndRejectsMalformedFlags) {
    const auto metadata = [](const char* schema) {
        output::Document document;
        document.SetObject();
        output::String(document, "schema", schema);
        return document;
    };
    auto legacy = metadata(failure_detail::LegacyFailureManifestSchema);
    EXPECT_FALSE(failure_detail::ConsumeBaselineObserved(legacy));
    EXPECT_EQ(legacy.MemberCount(), 1u);
    for (const bool value : {false, true}) {
        auto current = metadata(failure_detail::FailureManifestSchema);
        output::Boolean(current, "baseline_observed", value);
        EXPECT_EQ(failure_detail::ConsumeBaselineObserved(current), value);
        EXPECT_FALSE(current.HasMember("baseline_observed"));
        EXPECT_EQ(current.MemberCount(), 1u);
    }
    for (unsigned malformed = 0; malformed < 4; ++malformed) {
        auto current = metadata(failure_detail::FailureManifestSchema);
        if (malformed == 1) output::String(current, "baseline_observed", "true");
        if (malformed == 2) output::Integer(current, "baseline_observed", 1);
        if (malformed == 3) {
            output::Boolean(current, "baseline_observed", true);
            output::Boolean(current, "baseline_observed", false);
        }
        const auto count = current.MemberCount();
        EXPECT_THROW(failure_detail::ConsumeBaselineObserved(current), std::runtime_error);
        EXPECT_EQ(current.MemberCount(), count);
    }
    output::Boolean(legacy, "baseline_observed", false);
    EXPECT_THROW(failure_detail::ConsumeBaselineObserved(legacy), std::runtime_error);
    auto future = metadata("robo_dyna.self_contact_failure_fixture.v99");
    EXPECT_THROW(failure_detail::ConsumeBaselineObserved(future), std::runtime_error);
}

TEST(CandidateFailureFixtureValues, ObservedScopeDescribesCombinedProductionBudgetNotStandaloneReplay) {
    const std::string observed = failure_detail::ObservedBaselineScope;
    EXPECT_NE(observed.find("initial-root plus coverage work"), std::string::npos);
    EXPECT_NE(observed.find("production remaining budgets"), std::string::npos);
    EXPECT_NE(observed.find("not an exact production-budget replay"), std::string::npos);
    // An old default status alone still has no observed production provenance.
    prepared_replay::PairResult legacy;
    legacy.family = "failure";
    legacy.baseline_status = sct::NonlinearSeparationStatus::CertifiedSeparated;
    const auto document = prepared_replay::PairDocument(legacy);
    EXPECT_STREQ(document["baseline_depth_scope"].GetString(),
                 failure_detail::UnreportedBaselineDepthScope);
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
