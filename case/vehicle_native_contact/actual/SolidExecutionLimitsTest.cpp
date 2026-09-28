#include "SolidExecutionLimits.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::test {
TEST(SolidExecutionRequest, AbsentRequestPreservesExistingEightWorkerEnvelope) {
    for(const char* value:{nullptr,""}) {
        const auto limits=SolidExecutionLimits(value);
        EXPECT_EQ(limits.max_controlled_packet_blocks,8u);
        EXPECT_EQ(limits.max_device_bytes,192u<<20);EXPECT_EQ(limits.max_host_bytes,256u<<20);
    }
}
TEST(SolidExecutionRequest, OnlyExplicitSupportedRequestsSelectLargerComponentEnvelope) {
    for(const auto* value:{"4","8","16","32"}){
        const auto limits=SolidExecutionLimits(value);
        EXPECT_EQ(limits.max_controlled_packet_blocks,static_cast<unsigned>(std::stoi(value)));
        EXPECT_EQ(limits.max_device_bytes,(limits.max_controlled_packet_blocks>8?256u:192u)<<20);
        EXPECT_EQ(limits.max_host_bytes,(limits.max_controlled_packet_blocks>8?384u:256u)<<20);
        EXPECT_EQ(limits.max_nodes,tl::fea::solids::BatchLimits{}.max_nodes);
    }
    for(const auto* value:{"0","3","9","64","-8","8 ","8x"})
        EXPECT_THROW(SolidExecutionLimits(value),std::runtime_error);
}
}
