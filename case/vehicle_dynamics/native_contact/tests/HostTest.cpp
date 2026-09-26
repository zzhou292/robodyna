#include "case/vehicle_dynamics/native_contact/Contribution.h"
#include <gtest/gtest.h>
#include <cstring>
#include <type_traits>

namespace app = crash::cases::vehicle_dynamics::native_contact;
namespace n = app::native;
static_assert(!std::is_move_constructible_v<app::Contribution>);
static_assert(!std::is_copy_constructible_v<app::Contribution>);
static_assert(noexcept(std::declval<app::Contribution&>().Discard()));
static_assert(noexcept(std::declval<app::Contribution&>().Committed()));

TEST(NativeContributionHost, AdoptionRequiresAnActuallyInitializedTransaction) {
    EXPECT_THROW(app::Contribution::Adopt(nullptr), app::StageError);
    EXPECT_THROW(app::Contribution::Adopt(std::make_unique<n::Transaction>()), app::StageError);
    EXPECT_EQ(app::Contribution::bookkeeping_payload_bytes(), sizeof(app::Contribution));
}
TEST(NativeContributionHost, ErrorsOwnTheBorrowedMessageAndEveryNativeDiagnostic) {
    char message[] = "Native source stage rejected";
    n::TransactionReport report{n::TransactionStatus::NumericalFailure, message, 17, 29,
                               n::selection::Status::InvalidInput};
    n::TransactionSourceInfo source{71, 2, 3, 18, 7, 4, 8, true};
    n::TransactionDiagnostics diagnostics;
    diagnostics.candidate_rebuild_available = true;
    auto& candidate = diagnostics.candidate_rebuild;
    candidate.status = n::candidates::Status::ResourceLimit;
    candidate.failure_row = 19;
    candidate.stamp = {{71, 2}, 3, 4, 5, 6, 7};
    candidate.active_secondaries = 123;
    candidate.envelope_encounters = UINT64_C(2745000000);
    candidate.tasks = 11060785;
    candidate.pairs = 0; // Task cap was reached before pair counting.
    candidate.maximum_secondary_gap = .125;
    candidate.own_kernel_launches = 3;
    candidate.sort_calls = 1;
    candidate.scan_calls = 1;
    candidate.host_fences = 1;
    app::StageError error(app::Operation::SealCandidate, report, source, diagnostics);
    source = {};
    diagnostics = {};
    std::memset(message, 'x', sizeof(message) - 1);
    report.row = 99;
    EXPECT_NE(std::string(error.what()).find("Native source stage rejected"), std::string::npos);
    EXPECT_EQ(error.failure().operation, app::Operation::SealCandidate);
    EXPECT_EQ(error.failure().status, n::TransactionStatus::NumericalFailure);
    EXPECT_EQ(error.failure().row, 17u);
    EXPECT_EQ(error.failure().occurrence, 29u);
    EXPECT_EQ(error.failure().selection_status, n::selection::Status::InvalidInput);
    EXPECT_EQ(error.failure().source.source_id, 71u);
    EXPECT_TRUE(error.failure().source.available);
    EXPECT_TRUE(error.failure().diagnostics.candidate_rebuild_available);
    const auto& saved = error.failure().diagnostics.candidate_rebuild;
    EXPECT_EQ(saved.status, n::candidates::Status::ResourceLimit);
    EXPECT_EQ(saved.failure_row, 19u);
    EXPECT_EQ(saved.stamp.source.source, 71u);
    EXPECT_EQ(saved.stamp.attempt, 6u);
    EXPECT_EQ(saved.active_secondaries, 123u);
    EXPECT_EQ(saved.envelope_encounters, UINT64_C(2745000000));
    EXPECT_EQ(saved.tasks, 11060785u);
    EXPECT_EQ(saved.pairs, 0u);
    EXPECT_EQ(saved.maximum_secondary_gap, .125);
    EXPECT_EQ(saved.own_kernel_launches, 3u);
    EXPECT_EQ(saved.sort_calls, 1u);
    EXPECT_EQ(saved.scan_calls, 1u);
    EXPECT_EQ(saved.host_fences, 1u);
    app::StageError protocol(app::Operation::Assemble, report);
    EXPECT_FALSE(protocol.failure().source.available);
    EXPECT_FALSE(protocol.failure().diagnostics.candidate_rebuild_available);
}
