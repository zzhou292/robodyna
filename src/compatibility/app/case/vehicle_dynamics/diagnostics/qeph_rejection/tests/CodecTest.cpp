#include "Fixture.h"
#include "../WordArchive.h"
#include "../Export.h"
#include <limits>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::test {
TEST(QephRejectedCodec, FullOwnedRecordRoundtripsExactBitsAndTaggedFailureHistories) {
    for (auto policy : {tl::fea::ShellFailurePolicy::None, tl::fea::ShellFailurePolicy::ConstantAllPoints,
                        tl::fea::ShellFailurePolicy::Tab1AnyPoint}) {
        auto input = Captured().input;
        if (policy == tl::fea::ShellFailurePolicy::ConstantAllPoints) {
            input.accepted_failure = tl::fea::ShellBatchFailureState::Constant();
            input.accepted_failure.constant_points()[1] = {.25, .125, false};
        } else if (policy == tl::fea::ShellFailurePolicy::Tab1AnyPoint) {
            input.accepted_failure = tl::fea::ShellBatchFailureState::Tab1();
            input.accepted_failure.tab1_points()[1] = {1.25, 1., .125, 2, false};
        }
        input.failure_policy = policy;
        input.accepted_failure.current_force_point[2].stress[4] = -0.;
        input.curve_stress_pa[1023] = std::numeric_limits<double>::infinity();
        input.metadata.candidate.hourglass_viscous_work_increment = -0.;
        const auto encoded = Encode(input);
        const auto decoded = Decode(encoded);
        EXPECT_EQ(Encode(decoded), encoded);
        EXPECT_EQ(decoded.accepted_failure.policy(), policy);
        EXPECT_EQ(output::Bits(decoded.interval.position_endpoint[0].x), UINT64_C(0x7ff800000000cafe));
        EXPECT_EQ(output::Bits(decoded.accepted_failure.current_force_point[2].stress[4]), output::Bits(-0.));
        EXPECT_EQ(output::Bits(decoded.curve_stress_pa[1023]), output::Bits(std::numeric_limits<double>::infinity()));
        EXPECT_LT(encoded.size(), MaximumWords);
    }
}
TEST(QephRejectedCodec, MalformedWordStreamsAndIncompatibleHistoryCannotAcquireARecord) {
    const auto input = Captured().input;
    const auto good = Encode(input);
    auto bad = good; bad[0] ^= 1; EXPECT_THROW(Decode(bad), std::exception);
    bad = good; ++bad[1]; EXPECT_THROW(Decode(bad), std::exception);
    bad = good; bad.pop_back(); EXPECT_THROW(Decode(bad), std::exception);
    bad = good; bad.push_back(0); EXPECT_THROW(Decode(bad), std::exception);
    bad.assign(MaximumWords + 1, 0); EXPECT_THROW(Decode(bad), std::exception);
    auto unbound = input; unbound.element.reference.input.thickness *= 2;
    EXPECT_THROW(Encode(unbound), std::exception);
    std::vector<std::uint64_t> words{2}; codec::Reader boolean_reader(words); bool flag = false;
    EXPECT_THROW(boolean_reader(flag), std::exception);
    words[0] = UINT64_MAX; codec::Reader small_reader(words); std::uint32_t narrow = 0;
    EXPECT_THROW(small_reader(narrow), std::exception);
}
TEST(QephRejectedCodec, HostReplayUsesTheExistingOperatorBeforeAndAfterSerialization) {
    const auto input = Captured().input;
    native::RejectedReplayResult original, restored;
    ASSERT_TRUE(native::ReplayRejectedCandidate(input, &original));
    ASSERT_TRUE(native::ReplayRejectedCandidate(Decode(Encode(input)), &restored));
    EXPECT_EQ(original.operator_status, native::Status::kInvalidInput);
    EXPECT_EQ(restored.operator_status, original.operator_status);
    EXPECT_FALSE(original.force_available);
    EXPECT_FALSE(restored.force_available);
}
TEST(QephRejectedCapture, SourceAndAttemptMismatchCannotProduceACompleteSnapshot) {
    const auto good = Captured();
    for (unsigned fault = 0; fault < 4; ++fault) {
        auto state = good;
        auto requested = state.requested;
        native::RejectedCaptureReport report;
        report.status = native::RejectedCaptureStatus::Captured;
        report.metadata_available = true;
        report.metadata = state.metadata;
        if (fault == 0) ++requested.source.parent;
        if (fault == 1) requested.law = tl::fea::ShellSectionLaw::RigidSkin;
        if (fault == 2) ++report.metadata.candidate.attempt;
        if (fault == 3) report.metadata_available = false;
        const auto original = Original();
        FinishCapture(state, original, report, requested);
        EXPECT_FALSE(state.complete);
        EXPECT_EQ(state.status, native::RejectedCaptureStatus::InvalidSource);
        EXPECT_EQ(original.status, native::BatchStatus::ElementFailure);
        EXPECT_EQ(original.element_status, native::Status::kInvalidInput);
    }
}
TEST(QephRejectedCapture, UnsupportedAggregateAndFailedCaptureRetainOriginalReportSeparately) {
    for (auto status : {native::RejectedCaptureStatus::UnsupportedFailure,
                        native::RejectedCaptureStatus::DeviceFailure, native::RejectedCaptureStatus::ResourceLimit}) {
        CaptureState state;
        auto original = Original(); original.element = UINT32_MAX;
        native::RejectedCaptureReport report;
        report.status = status;
        report.message = "Specific capture failure";
        report.metadata_available = true;
        report.metadata.original = {original.status, original.element, original.node,
            original.element_status, original.nodal_status};
        FinishCapture(state, original, report, {});
        EXPECT_TRUE(state.seen);
        EXPECT_FALSE(state.complete);
        EXPECT_EQ(state.status, status);
        EXPECT_EQ(state.original.status, original.status);
        EXPECT_EQ(state.original.element, UINT32_MAX);
        EXPECT_STREQ(state.message.data(), "Specific capture failure");
        EXPECT_FALSE(EncodeMetadata(state.metadata).empty());
    }
}
}
