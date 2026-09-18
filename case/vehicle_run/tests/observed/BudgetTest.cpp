#include "RunAccess.h"
#include "Contribution.h"
#include <gtest/gtest.h>
#include <limits>

namespace crash::cases::vehicle_run::observed {
namespace {
Forecast Input() {
    Forecast forecast;
    forecast.contact.self_contact.emplace();
    forecast.complete_host_bytes = 1701;
    forecast.complete_archive_bytes = 902;
    forecast.caps.host_bytes = forecast.complete_host_bytes +
        fixture::FailureCaptureHostCap + ContributionHostReserve;
    forecast.caps.archive_bytes = forecast.complete_archive_bytes +
        fixture::FailureFixtureArchiveCap;
    return forecast;
}
TEST(ObservedControllerBudget, ExactExistingCapsAdmitAllCompanionsWithoutDeviceGrowth) {
    const auto forecast = Input();
    const auto result = Preflight(forecast);
    EXPECT_EQ(result.host_bytes, forecast.caps.host_bytes);
    EXPECT_EQ(result.archive_bytes, forecast.caps.archive_bytes);
    EXPECT_EQ(forecast.contact.device_bytes, 0u);
}
TEST(ObservedControllerBudget, OneByteUnderEitherCompleteCapRejects) {
    auto forecast = Input();
    --forecast.caps.host_bytes;
    EXPECT_THROW(Preflight(forecast), std::exception);
    forecast = Input();
    --forecast.caps.archive_bytes;
    EXPECT_THROW(Preflight(forecast), std::exception);
}
TEST(ObservedControllerBudget, OverflowAndMissingSelfProfileRejectBeforeAllocation) {
    auto forecast = Input();
    forecast.complete_host_bytes = forecast.caps.host_bytes =
        std::numeric_limits<std::size_t>::max();
    EXPECT_THROW(Preflight(forecast), std::exception);
    forecast = Input();
    forecast.complete_archive_bytes = forecast.caps.archive_bytes =
        std::numeric_limits<std::size_t>::max();
    EXPECT_THROW(Preflight(forecast), std::exception);
    forecast = Input();
    forecast.contact.self_contact.reset();
    EXPECT_THROW(Preflight(forecast), std::exception);
}
} // namespace
} // namespace crash::cases::vehicle_run::observed
