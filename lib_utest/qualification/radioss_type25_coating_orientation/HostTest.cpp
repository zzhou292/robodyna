// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
namespace coating_orientation_test {
TEST(CoatingOrientationHost, CompleteNativeDeterminantAndSignMatchForEveryPacket) {
    const auto packets = Cases();
    ASSERT_EQ(packets.size(), 64u);
    unsigned forward = 0, reversed = 0, zero = 0;
    for (std::size_t i = 0; i < packets.size(); ++i) {
        SCOPED_TRACE(i);
        const auto before = packets[i];
        s::NativeCoatingOrientationResult result;
        ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packets[i], &result), s::Status::Ok);
        Same(result, Oracle(packets[i]));
        SameInput(packets[i], before);
        forward += result.orientation == s::CoatingOrientation::Forward;
        reversed += result.orientation == s::CoatingOrientation::Reversed;
        zero += result.center_triangle_determinant == 0;
    }
    EXPECT_GT(forward, 0u);
    EXPECT_GT(reversed, 0u);
    EXPECT_GT(zero, 0u);
}
TEST(CoatingOrientationHost, OriginalStrictZeroBoundaryKeepsForwardAndCornerOrderReversesSign) {
    auto packet = Packet();
    s::NativeCoatingOrientationResult first, second;
    ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packet, &first), s::Status::Ok);
    std::swap(packet.segment_slots[0], packet.segment_slots[1]);
    ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packet, &second), s::Status::Ok);
    Same(second, Oracle(packet));
    EXPECT_NE(first.orientation, second.orientation);
    for (unsigned i = 0; i < packet.node_count; ++i) packet.positions[i].z = 0.;
    ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packet, &second), s::Status::Ok);
    Same(second, Oracle(packet));
    EXPECT_EQ(second.orientation, s::CoatingOrientation::Forward);
    EXPECT_EQ(second.center_triangle_determinant, 0.);
}
TEST(CoatingOrientationHost, InvalidConsumedFieldsAndArithmeticPreserveOutputAndAllowRetry) {
    const s::NativeCoatingOrientationResult seed{s::CoatingOrientation::Reversed, -17.25};
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    for (unsigned fault = 0; fault < 5; ++fault) {
        auto packet = Packet();
        auto output = seed;
        auto expected = s::Status::InvalidInput;
        switch (fault) {
        case 0:
            packet.node_count = 6;
            expected = s::Status::UnsupportedProfile;
            break;
        case 1:
            packet.segment_slots[0] = packet.node_count;
            break;
        case 2:
            packet.positions[7].z = nan;
            break;
        case 3:
            for (unsigned i = 0; i < packet.node_count; ++i)
                packet.positions[i].x = std::numeric_limits<double>::max();
            expected = s::Status::NonfiniteResult;
            break;
        case 4:
            for (unsigned i = 0; i < packet.node_count; ++i) {
                packet.positions[i].x *= 1e160;
                packet.positions[i].y *= 1e160;
            }
            expected = s::Status::NonfiniteResult;
            break;
        }
        EXPECT_EQ(s::EvaluateNativeCoatingOrientation(packet, &output), expected);
        Same(output, seed);
    }
    auto packet = Packet();
    EXPECT_EQ(s::EvaluateNativeCoatingOrientation(packet, nullptr), s::Status::InvalidInput);
    auto output = seed;
    ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packet, &output), s::Status::Ok);
    Same(output, Oracle(packet));
}
TEST(CoatingOrientationHost, UnusedTailAndCountAdmissionDoNotInspectPoisonedSlots) {
    for (unsigned count : {8u, 10u, 16u}) {
        auto packet = Packet(count);
        s::NativeCoatingOrientationResult output;
        ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packet, &output), s::Status::Ok);
        Same(output, Oracle(packet));
    }
    auto packet = Packet();
    packet.node_count = std::numeric_limits<unsigned>::max();
    for (auto& point : packet.positions) point = {NAN, NAN, NAN};
    s::NativeCoatingOrientationResult output{s::CoatingOrientation::Forward, 12.};
    const auto before = output;
    EXPECT_EQ(s::EvaluateNativeCoatingOrientation(packet, &output), s::Status::UnsupportedProfile);
    Same(output, before);
}
}
