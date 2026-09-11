#include "PackingFixture.h"
#include "../Internal.h"

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionPacking, InclusiveReservationsExactBoundaryOverflowAndCapacity) {
    Limits limits;
    const auto forecast = detail::ForecastPayload(1234,512,3,7,3,limits);
    EXPECT_EQ(forecast.total_bytes,forecast.source_bytes + forecast.fixed_bytes +
              forecast.packing_bytes + forecast.native_reservation);
    detail::Packing packing;
    packing.Reserve(3,7,3);
    EXPECT_LE(packing.capacity_bytes(),forecast.packing_bytes);
    limits.host_bytes = forecast.total_bytes - 1;
    EXPECT_THROW(detail::ForecastPayload(1234,512,3,7,3,limits),std::runtime_error);
    limits.host_bytes = forecast.total_bytes;
    EXPECT_EQ(detail::ForecastPayload(1234,512,3,7,3,limits).total_bytes,forecast.total_bytes);
    EXPECT_THROW(detail::ForecastPayload(SIZE_MAX,512,3,7,3,limits),std::runtime_error);
    EXPECT_THROW(detail::ForecastPayload(1234,512,1025,7,3,limits),std::runtime_error);
    limits.catalog.max_startup_scratch_bytes = 0;
    EXPECT_THROW(detail::ForecastPayload(1234,512,3,7,3,limits),std::runtime_error);
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
