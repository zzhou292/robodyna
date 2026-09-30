#include "Source.h"

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
TEST(VehicleSelfContactSource, ExactStartupCapLateSourceFailureAndRetainedRetry) {
    const auto& physical = Execution().physical();
    auto limits = contact::SelfContactSurfaceLimits::Vehicle();
    const auto input = Input(Inventory().centered);
    ASSERT_GT(input.parent_count, 0u);
    const auto preflight = contact::SelfContactSurfaceBinding::Preflight(physical,input,limits);
    ASSERT_EQ(preflight.report.status,contact::SelfContactSurfaceStatus::Ok);
    limits.max_host_bytes = preflight.forecast.startup_payload_bytes;
    EXPECT_EQ(contact::SelfContactSurfaceBinding::Preflight(physical,input,limits).report.status,
        contact::SelfContactSurfaceStatus::Ok);
    contact::SelfContactSurfaceBinding retried;
    auto short_limit = limits;
    --short_limit.max_host_bytes;
    EXPECT_EQ(retried.Initialize(physical,input,short_limit).status,contact::SelfContactSurfaceStatus::ResourceLimit);
    EXPECT_FALSE(retried.prepared());
    auto altered = Inventory().centered;
    ++altered.back().source_parent_id;
    const auto rejected = retried.Initialize(physical,Input(altered),limits);
    EXPECT_EQ(rejected.status,contact::SelfContactSurfaceStatus::IdentityMismatch);
    EXPECT_EQ(rejected.selection,altered.size()-1);
    EXPECT_FALSE(retried.prepared());
    {
        const auto temporary_physical = physical;
        ASSERT_EQ(retried.Initialize(temporary_physical,input,limits).status,contact::SelfContactSurfaceStatus::Ok);
    }
    EXPECT_TRUE(retried.MatchesPhysical(physical));
    const auto retained = retried;
    EXPECT_TRUE(retained.SharesStorage(retried));
    EXPECT_EQ(retained.parents().size(),input.parent_count);
    EXPECT_EQ(retained.forecast().startup_payload_bytes,limits.max_host_bytes);
}
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
