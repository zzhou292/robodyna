// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace wall_assembly_test {
TEST(MappedWallAssemblyInputsHost, SparseRowsRigidZeroAndSignedZeroMatchFrozenInverseBits) {
  for (unsigned count : {7u, 127u, 128u, 129u, 263u, 33001u}) {
    SCOPED_TRACE(count);
    Packet serial(count), parallel(count);
    Serial(serial);
    Staged(parallel);
    ASSERT_TRUE(parallel.summary.points_admitted);
    Same(parallel, serial);
    EXPECT_EQ(a::Blocks(count), std::min(1u+(count-1)/a::Threads, a::MaximumBlocks));
  }
}
TEST(MappedWallAssemblyInputsHost, FirstCompactRowAndWithinRowPriorityPreserveExactPartialPrefix) {
  for (unsigned fault = 0; fault < 12; ++fault) {
    SCOPED_TRACE(fault);
    Packet serial, parallel;
    Fault(serial, fault);
    Fault(parallel, fault);
    Serial(serial);
    Staged(parallel);
    ASSERT_FALSE(parallel.summary.points_admitted);
    Same(parallel, serial);
    if (fault < 8 || fault >= 10) {
      const auto row = fault >= 10 ? 0u : fault >= 6 ? 5u : parallel.count-1;
      EXPECT_EQ(parallel.storage.control.node, parallel.nodes[row].node);
      const bool geometry = fault == 5 || fault == 7 || fault == 11;
      EXPECT_EQ(parallel.storage.control.status,
          geometry ? c::NodalWallDeviceStatus::GeometryFailure : c::NodalWallDeviceStatus::InvalidMass);
      const auto prefix = row+(geometry ? 1u : 0u);
      for (unsigned i = prefix; i < parallel.count; ++i) EXPECT_EQ(Bits(parallel.copied[i]), Bits(-91.25));
    }
    serial.Reset();
    parallel.Reset();
    Serial(serial);
    Staged(parallel);
    ASSERT_TRUE(parallel.summary.points_admitted);
    Same(parallel, serial);
  }
}
TEST(MappedWallAssemblyInputsHost, LaterEpochDoesNotConsumeCoordinatesAndHeaderLeavesInverseUntouched) {
  Packet serial, parallel;
  for (auto* p : {&serial, &parallel}) {
    p->epoch = 7;
    p->Reset();
    std::fill(p->position.begin(), p->position.end(), NAN);
    for (auto& x : p->initial) x = {NAN, INFINITY, -INFINITY};
  }
  Serial(serial);
  Staged(parallel);
  ASSERT_TRUE(parallel.summary.points_admitted);
  Same(parallel, serial);
  for (auto* p : {&serial, &parallel}) {
    p->Reset();
    p->bounds.valid = false;
    p->storage.model.initial_position = nullptr;
    p->inverse.clear();
    p->fixed.clear();
    p->translation.clear();
  }
  Serial(serial);
  Staged(parallel);
  EXPECT_EQ(parallel.storage.control.status, c::NodalWallDeviceStatus::AssemblyFailure);
  Same(parallel, serial);
  for (double inverse : parallel.copied) EXPECT_EQ(Bits(inverse), Bits(-91.25));
}
} // namespace wall_assembly_test
