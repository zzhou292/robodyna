#include "Fixture.h"

#include <gtest/gtest.h>
#include <array>

namespace robodyna::assembly_compat {
TEST(AssemblyCompatibility, MixedSetupPreservesCollectionOrderAndOffsets) {
    Fixture f;
    ASSERT_EQ(f.assembly->GetBodies().size(), 2u);
    EXPECT_EQ(f.assembly->GetBodies()[0], f.body);
    EXPECT_EQ(f.assembly->GetBodies()[1], f.fixed);
    ASSERT_EQ(f.assembly->GetShafts().size(), 1u);
    ASSERT_EQ(f.assembly->GetMeshes().size(), 1u);
    ASSERT_EQ(f.assembly->GetLinks().size(), 1u);
    EXPECT_EQ(f.assembly->GetShafts()[0], f.shaft);
    EXPECT_EQ(f.assembly->GetMeshes()[0], f.mesh);
    EXPECT_EQ(f.assembly->GetLinks()[0], f.link);
    f.assembly->SetOffset_x(11);
    f.assembly->SetOffset_w(17);
    f.assembly->SetOffset_L(23);
    f.assembly->Setup();
    EXPECT_EQ(f.assembly->GetNumBodiesActive(), 1u);
    EXPECT_EQ(f.assembly->GetNumBodiesFixed(), 1u);
    EXPECT_EQ(f.assembly->GetNumShafts(), 1u);
    EXPECT_EQ(f.assembly->GetNumMeshes(), 1u);
    EXPECT_EQ(f.assembly->GetNumLinksActive(), 1u);
    EXPECT_EQ(f.assembly->GetNumCoordsPosLevel(), 11u);  // body 7 + shaft 1 + node 3
    EXPECT_EQ(f.assembly->GetNumCoordsVelLevel(), 10u);  // body 6 + shaft 1 + node 3
    EXPECT_EQ(f.assembly->GetNumConstraintsBilateral(), 3u);
    EXPECT_EQ(f.body->GetOffset_x(), 11u);
    EXPECT_EQ(f.body->GetOffset_w(), 17u);
    EXPECT_EQ(f.shaft->GetOffset_x(), 18u);
    EXPECT_EQ(f.shaft->GetOffset_w(), 23u);
    EXPECT_EQ(f.link->GetOffset_L(), 23u);
    EXPECT_EQ(f.mesh->GetOffset_x(), 19u);
    EXPECT_EQ(f.mesh->GetOffset_w(), 24u);
    EXPECT_EQ(f.mesh->GetOffset_L(), 26u);
    EXPECT_EQ(f.moving_node->NodeGetOffsetPosLevel(), 19u);
    EXPECT_EQ(f.moving_node->NodeGetOffsetVelLevel(), 24u);
    f.assembly->Setup();
    EXPECT_EQ(f.assembly->GetNumCoordsVelLevel(), 10u);
    EXPECT_EQ(f.moving_node->NodeGetOffsetVelLevel(), 24u);
}

TEST(AssemblyCompatibility, NestedRemovalDetachesRecursivelyAndPreservesSharedObjects) {
    Fixture f;
    auto child = std::make_shared<chrono::ChAssembly>();
    auto child_body = std::make_shared<chrono::ChBody>();
    f.assembly->Add(child);
    child->Add(child_body);
    f.system.Setup();
    f.system.Update(.125, chrono::UpdateFlags::UPDATE_ALL);
    const std::array<chrono::ChPhysicsItem*, 7> items{
        f.assembly.get(), f.body.get(), f.shaft.get(), f.mesh.get(), f.link.get(), child.get(), child_body.get()};
    for (auto* item : items) {
        EXPECT_EQ(item->GetSystem(), &f.system);
        EXPECT_DOUBLE_EQ(item->GetChTime(), .125);
    }
    EXPECT_EQ(f.system.GetNumSteps(), 0u);  // Update is not a second dynamics step.
    f.system.Remove(f.assembly);
    for (auto* item : items)
        EXPECT_EQ(item->GetSystem(), nullptr);
    EXPECT_EQ(child->GetBodies()[0], child_body);
    f.assembly->Clear();
    EXPECT_TRUE(f.assembly->GetBodies().empty());
    EXPECT_TRUE(f.assembly->GetMeshes().empty());
    EXPECT_EQ(child->GetBodies()[0], child_body);  // External owner keeps the detached child assembly.
    std::weak_ptr<chrono::ChBody> weak = child_body;
    child_body.reset();
    EXPECT_FALSE(weak.expired());
    child->Clear();
    EXPECT_TRUE(weak.expired());
}

TEST(AssemblyCompatibility, ClearRetainsPendingBatchUntilNextSetup) {
    Fixture f;
    auto queued = std::make_shared<chrono::ChBody>();
    f.assembly->AddBatch(queued);
    EXPECT_EQ(queued->GetSystem(), nullptr);
    f.assembly->Clear();
    EXPECT_TRUE(f.assembly->GetBodies().empty());
    EXPECT_EQ(f.assembly->GetNumCoordsVelLevel(), 0u);
    // This is inherited behavior: Clear does not clear batch_to_insert.
    f.assembly->Setup();
    ASSERT_EQ(f.assembly->GetBodies().size(), 1u);
    EXPECT_EQ(f.assembly->GetBodies()[0], queued);
    EXPECT_EQ(queued->GetSystem(), &f.system);
    EXPECT_EQ(f.assembly->GetNumCoordsVelLevel(), 6u);
    f.assembly->Setup();
    EXPECT_EQ(f.assembly->GetBodies().size(), 1u);
}
}  // namespace robodyna::assembly_compat
