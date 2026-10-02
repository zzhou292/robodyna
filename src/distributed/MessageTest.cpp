#include "chrono_synchrono/flatbuffer/SynFlatBuffersManager.h"
#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

#include <gtest/gtest.h>

TEST(DistributedMessages, RetainedSchemaPreservesRoutingAndQuitState) {
    using namespace chrono::synchrono;
    SynFlatBuffersManager writer;
    writer.Reset();
    writer.AddMessage(std::make_shared<SynSimulationMessage>(AgentKey(7, 3), AgentKey(8, 4), true));
    writer.Finish();
    auto bytes = writer.ToMessageBuffer();

    SynFlatBuffersManager reader;
    SynMessageList messages;
    reader.ProcessBuffer(bytes, messages);
    ASSERT_EQ(messages.size(), 1u);
    auto restored = std::dynamic_pointer_cast<SynSimulationMessage>(messages.front());
    ASSERT_NE(restored, nullptr);
    EXPECT_EQ(restored->GetSourceKey().GetNodeID(), 7);
    EXPECT_EQ(restored->GetSourceKey().GetAgentID(), 3);
    EXPECT_EQ(restored->GetDestinationKey().GetNodeID(), 8);
    EXPECT_EQ(restored->GetDestinationKey().GetAgentID(), 4);
    EXPECT_TRUE(restored->m_quit_sim);

    writer.Reset();
    writer.Finish();
    auto empty = writer.ToMessageBuffer();
    messages.clear();
    reader.ProcessBuffer(empty, messages);
    EXPECT_TRUE(messages.empty());
    EXPECT_LT(empty.size(), bytes.size());
}
