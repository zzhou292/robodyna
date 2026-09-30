#include "OriginalSupport.h"

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionOriginal, InclusiveCapLatePhysicalRejectionAndSharedLifetimeRetry) {
    auto value = Actual();
    const auto* before = value.execution().parents().data();
    Limits limits;
    limits.host_bytes = value.forecast().total_bytes - 1;
    EXPECT_THROW(value = VehicleShellExecution::Prepare(value.model(),limits),std::runtime_error);
    EXPECT_EQ(value.execution().parents().data(),before);
    ++limits.host_bytes;
    EXPECT_EQ(VehicleShellExecution::Preflight(value.model(),limits).total_bytes,limits.host_bytes);
    // The last native immutable join fails after catalog/failure/PART packing.
    limits = {};
    limits.physical.max_host_bytes = value.physical().owned_payload_bytes() - 1;
    EXPECT_THROW(value = VehicleShellExecution::Prepare(value.model(),limits),std::runtime_error);
    EXPECT_EQ(value.execution().parents().data(),before);
    ++limits.physical.max_host_bytes;
    limits.host_bytes = VehicleShellExecution::Preflight(value.model(),limits).total_bytes;
    EXPECT_NO_THROW(value = VehicleShellExecution::Prepare(value.model(),limits));
    EXPECT_TRUE(value.catalog().SameScope(Actual().catalog()));
    EXPECT_TRUE(value.physical().coefficients()->Matches(Actual().model().coefficients()));
    const auto moved = [=] {
        auto temporary = value;
        return temporary;
    }();
    EXPECT_EQ(moved.execution().parents().data(),value.execution().parents().data());
    EXPECT_EQ(&moved.resolution(),&value.resolution());
    const auto last = moved.catalog().parent_count()-1;
    EXPECT_EQ(moved.catalog().parent(last)->source_parent_id,moved.resolution().parents()[last].source_parent_id);
    SameFailure(*moved.failure().parent(last),*moved.resolution().native_parent(last));
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
