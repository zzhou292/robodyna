#include "Input.h"
#include "lib_src/elements/type25/Type25BatchArena.h"

namespace type25_batch_test {
TEST(Type25VehicleLayout,ExplicitSourceAndMaximumCountsHaveBoundedRebasedTailAndLegacyRejection) {
  namespace b=spring::batch_detail;
  for(const auto sizes:std::array<std::array<std::size_t,2>,2>{{{2828,359785},{4096,524288}}}) {
    b::ArenaLayout layout;layout.bytes=123;
    EXPECT_FALSE(b::MakeLayout(64,sizes[0],sizes[1],spring::VehicleCapacity.batch_device_bytes,layout));EXPECT_EQ(layout.bytes,123u);
    ASSERT_TRUE(b::MakeLayout(64,sizes[0],sizes[1],spring::VehicleCapacity.batch_device_bytes,layout,spring::CapacityProfile::Vehicle));
    EXPECT_LT(layout.bytes,25u*1024u*1024u);EXPECT_EQ(layout.nodes.count,sizes[1]);EXPECT_EQ(layout.slab[1].count,sizes[0]);
    tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(layout.bytes));const auto header=b::RebasedHeader(arena.data(),layout);
    EXPECT_EQ(reinterpret_cast<const unsigned char*>(header.model.nodes+sizes[1])-static_cast<const unsigned char*>(arena.data()),layout.nodes.offset+layout.nodes.bytes);
    EXPECT_EQ(reinterpret_cast<const unsigned char*>(header.slab[1].element+sizes[0])-static_cast<const unsigned char*>(arena.data()),layout.slab[1].offset+layout.slab[1].bytes);
    b::ArenaLayout output;output.bytes=713;
    EXPECT_FALSE(b::MakeLayout(64,sizes[0],sizes[1],layout.bytes-1,output,spring::CapacityProfile::Vehicle));EXPECT_EQ(output.bytes,713u);
    EXPECT_TRUE(b::MakeLayout(64,sizes[0],sizes[1],layout.bytes,output,spring::CapacityProfile::Vehicle));
    RecordProperty("maximum_arena_bytes",std::to_string(layout.bytes));
  }
  b::ArenaLayout out;out.bytes=117;
  for(const auto c:{std::size_t{4097},SIZE_MAX}) {
    EXPECT_FALSE(b::MakeLayout(1,c,524288,spring::VehicleCapacity.batch_device_bytes,out,spring::CapacityProfile::Vehicle));EXPECT_EQ(out.bytes,117u);
  }
  EXPECT_FALSE(b::MakeLayout(65,4096,524288,spring::VehicleCapacity.batch_device_bytes,out,spring::CapacityProfile::Vehicle));
  EXPECT_FALSE(b::MakeLayout(64,4096,524289,spring::VehicleCapacity.batch_device_bytes,out,spring::CapacityProfile::Vehicle));
  EXPECT_FALSE(b::MakeLayout(64,4096,524288,SIZE_MAX,out,spring::CapacityProfile::Vehicle));EXPECT_EQ(out.bytes,117u);
}
} // namespace type25_batch_test
