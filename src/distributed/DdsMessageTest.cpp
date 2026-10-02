#include "chrono_synchrono/communication/dds/idl/SynDDSMessagePubSubTypes.h"
#include "fastrtps/rtps/common/SerializedPayload.h"

#include <gtest/gtest.h>
#include <vector>

TEST(DistributedDds, OriginalGeneratedTypeRoundTripsRealCdrPayload) {
    SynDDSMessage original;
    original.rank(0xabcdefu);
    original.data(std::vector<uint8_t>{0, 1, 127, 128, 255, 7});
    SynDDSMessagePubSubType type;
    const auto size = type.getSerializedSizeProvider(&original)();
    eprosima::fastrtps::rtps::SerializedPayload_t payload(size);
    ASSERT_TRUE(type.serialize(&original, &payload));
    EXPECT_EQ(payload.length, size);
    SynDDSMessage restored;
    ASSERT_TRUE(type.deserialize(&payload, &restored));
    EXPECT_EQ(restored.rank(), original.rank());
    EXPECT_EQ(restored.data(), original.data());

    original.rank(0);
    original.data(std::vector<uint8_t>{});
    ASSERT_TRUE(type.serialize(&original, &payload));
    ASSERT_TRUE(type.deserialize(&payload, &restored));
    EXPECT_EQ(restored.rank(), 0u);
    EXPECT_TRUE(restored.data().empty());
}

TEST(DistributedDds, OriginalSerializerRejectsInsufficientPayloadCapacity) {
    SynDDSMessage original;
    original.rank(3);
    original.data(std::vector<uint8_t>(64, 42));
    SynDDSMessagePubSubType type;
    // Enough for the original encapsulation header, too small for rank+sequence.
    eprosima::fastrtps::rtps::SerializedPayload_t payload(4);
    EXPECT_FALSE(type.serialize(&original, &payload));
}
