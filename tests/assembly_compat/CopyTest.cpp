#include "Fixture.h"

#include <gtest/gtest.h>

namespace robodyna::assembly_compat {
TEST(AssemblyCompatibility, CopyAndCloneCopyCountersButNotChildCollections) {
    Fixture f;
    f.assembly->Setup();
    chrono::ChAssembly copied(*f.assembly);
    std::unique_ptr<chrono::ChAssembly> cloned(f.assembly->Clone());
    for (auto* value : {&copied, cloned.get()}) {
        EXPECT_EQ(value->GetSystem(), nullptr);
        EXPECT_TRUE(value->GetBodies().empty());
        EXPECT_TRUE(value->GetMeshes().empty());
        EXPECT_TRUE(value->GetShafts().empty());
        EXPECT_TRUE(value->GetLinks().empty());
        EXPECT_TRUE(value->GetOtherPhysicsItems().empty());
        EXPECT_EQ(value->GetNumBodies(), 2u);
        EXPECT_EQ(value->GetNumCoordsPosLevel(), 11u);
        EXPECT_EQ(value->GetNumCoordsVelLevel(), 10u);
        value->Setup();
        EXPECT_EQ(value->GetNumBodies(), 0u);
        EXPECT_EQ(value->GetNumCoordsPosLevel(), 0u);
        EXPECT_EQ(value->GetNumCoordsVelLevel(), 0u);
    }
    EXPECT_EQ(f.body->GetSystem(), &f.system);
    EXPECT_EQ(f.assembly->GetBodies()[0], f.body);
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(chrono::ChAssembly)), "ChAssembly");
}

TEST(AssemblyCompatibility, AssignmentAndAdlSwapKeepTheirOriginalChildLists) {
    Fixture f;
    f.assembly->Setup();
    auto other = std::make_shared<chrono::ChAssembly>();
    auto other_body = std::make_shared<chrono::ChBody>();
    other_body->SetFixed(true);
    f.system.Add(other);
    other->Add(other_body);
    other->Setup();
    using std::swap;
    swap(*f.assembly, *other);  // Must still select the inherited ADL overload after relocation.
    EXPECT_EQ(f.assembly->GetBodies().size(), 2u);
    EXPECT_EQ(other->GetBodies().size(), 1u);
    EXPECT_EQ(f.assembly->GetNumCoordsVelLevel(), 0u);
    EXPECT_EQ(other->GetNumCoordsVelLevel(), 10u);
    f.assembly->Setup();
    other->Setup();
    *other = *f.assembly;
    // Existing assignment copies statistics, without replacing the destination topology.
    ASSERT_EQ(other->GetBodies().size(), 1u);
    EXPECT_EQ(other->GetBodies()[0], other_body);
    EXPECT_TRUE(other->GetMeshes().empty());
    EXPECT_EQ(other->GetNumCoordsVelLevel(), 10u);
    EXPECT_EQ(other->GetSystem(), &f.system);
    EXPECT_EQ(other_body->GetSystem(), &f.system);
    other->Setup();
    EXPECT_EQ(other->GetNumCoordsVelLevel(), 0u);
    EXPECT_EQ(f.assembly->GetNumCoordsVelLevel(), 10u);
}
}  // namespace robodyna::assembly_compat
