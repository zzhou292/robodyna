#include "InventoryFixture.h"
#include <gtest/gtest.h>
using namespace candidate_test;
// This target links only the production CUDA library, no Fortran/oracle target.
TEST(NativeCandidateConsumer,IndependentProductionInventoryLink) {
  Scene scene;c::Inventory inventory;
  ASSERT_EQ(inventory.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  ASSERT_EQ(inventory.Stage(scene.Current()),c::Status::Ok);
  EXPECT_TRUE(inventory.IsCurrent(inventory.view()));EXPECT_GT(inventory.view().pair_count(),0u);
}
