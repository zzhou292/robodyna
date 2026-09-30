// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace qbat_measurement_test {
static_assert(sizeof(b::Storage)==656+8,"Only one pointer is added to the device header");
static_assert(sizeof(b::Layout)==456+24,"One counted region is added to the host layout");
TEST(QbatMeasurementHost,MappedTailHasExactInclusiveCapAndUnmappedHasNoOperandRows) {
  for (std::size_t parents : {1u,4u,129u,4250u}) {
    b::Layout mapped,plain;
    ASSERT_TRUE(mapped.InitializeMapped(parents,376930,0,fe::MaxVehicleShellResidentDeviceBytes));
    ASSERT_TRUE(plain.Initialize(parents,376930,0,fe::MaxVehicleShellResidentDeviceBytes));
    EXPECT_EQ(mapped.assembly.measurement.count,parents);
    EXPECT_EQ(mapped.assembly.measurement.bytes,168*parents);
    EXPECT_EQ(mapped.assembly.measurement.offset+168*parents,mapped.bytes);
    EXPECT_EQ(plain.assembly.measurement.count,0u);
    EXPECT_EQ(plain.assembly.bytes,0u);
    const auto exact=mapped.bytes;
    EXPECT_FALSE(mapped.InitializeMapped(parents,376930,0,exact-1));
    EXPECT_EQ(mapped.bytes,exact);
    EXPECT_TRUE(mapped.InitializeMapped(parents,376930,0,exact));
    EXPECT_FALSE(mapped.InitializeMapped(0,376930,0,exact));
    EXPECT_EQ(mapped.bytes,exact);
    EXPECT_FALSE(mapped.InitializeMapped(SIZE_MAX,376930,0,SIZE_MAX));
    EXPECT_EQ(mapped.bytes,exact);
    if (parents==4250) {
      // Complete baseline layout is independently recorded by the frozen
      // gather qualifier, not reconstructed from the new packet dimensions.
      EXPECT_EQ(mapped.bytes,82042528u+714008u);
      EXPECT_EQ(plain.bytes,82042528u-29024792u+8u);
    }
  }
}
TEST(QbatMeasurementHost,TypedStartupConstructsOnlyMappedOperandsAndRebasesInsideArena) {
  Fixture f;
  ASSERT_FALSE(HasFailure());
  EXPECT_EQ(reinterpret_cast<unsigned char*>(f.host->assembly.measurement),
      static_cast<unsigned char*>(f.arena.data())+f.layout.assembly.measurement.offset);
  auto rebased=f.layout.Rebase(*f.host,f.arena.data());
  EXPECT_EQ(rebased.assembly.measurement,f.host->assembly.measurement);
  b::Layout plain;
  ASSERT_TRUE(plain.Initialize(1,4,0,1u<<20));
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(plain.bytes));
  auto* storage=plain.Construct(arena);
  ASSERT_NE(storage,nullptr);
  EXPECT_EQ(storage->assembly.measurement,nullptr);
  EXPECT_EQ(plain.Rebase(*storage,arena.data()).assembly.measurement,nullptr);
}
} // namespace qbat_measurement_test
