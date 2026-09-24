#include "CandidateFailureConeAssertions.h"
#include <cstdlib>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
TEST(CandidateFailureConeReplay, PinnedAffineSharedVertexPairRetainsCompleteLocalPolicy) {
    const auto* path = std::getenv("ROBO_SELF_CONTACT_CONE_FAILURE_MANIFEST");
    ASSERT_TRUE(path && *path) << "Set ROBO_SELF_CONTACT_CONE_FAILURE_MANIFEST to the gate8 failure.json";
    cone_replay::CheckPinnedPolicy(path,
        "3b2ab9990ad1dbf5c98a27c28420d2fb9e0847b2226337eb519a69e8d3559da9",
        "0bf7a5002eb07d8c9367de2d8174410a52566769d1eed5eabf74f00277cabed7");
}
}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
