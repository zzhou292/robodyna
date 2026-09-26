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
    app::StageError error(app::Operation::SealCandidate, report);
    std::memset(message, 'x', sizeof(message) - 1);
    report.row = 99;
    EXPECT_NE(std::string(error.what()).find("Native source stage rejected"), std::string::npos);
    EXPECT_EQ(error.failure().operation, app::Operation::SealCandidate);
    EXPECT_EQ(error.failure().status, n::TransactionStatus::NumericalFailure);
    EXPECT_EQ(error.failure().row, 17u);
    EXPECT_EQ(error.failure().occurrence, 29u);
    EXPECT_EQ(error.failure().selection_status, n::selection::Status::InvalidInput);
}
