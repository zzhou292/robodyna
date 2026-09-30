// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace wall_response_test {
TEST(MappedWallResponseHost, OrderedRigidTracesAndOrdinaryMaxMatchEveryBitAcrossBlocks) {
  for (unsigned count : {7u, 127u, 128u, 129u, 263u, 33001u}) {
    for (unsigned groups : {0u, 7u, 1024u}) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(groups);
      Packet serial(count, groups), staged(count, groups);
      Serial(serial); Staged(staged);
      ASSERT_EQ(staged.storage.control.status, c::NodalWallDeviceStatus::Ok);
      Same(staged, serial);
    }
  }
}
TEST(MappedWallResponseHost, CompleteOldFailurePriorityPartialPrefixAndRetry) {
  for (unsigned fault = 0; fault < 13; ++fault) {
    SCOPED_TRACE(fault);
    Packet serial, staged;
    Fault(serial, fault); Fault(staged, fault);
    Serial(serial); Staged(staged);
    ASSERT_NE(staged.storage.control.status, c::NodalWallDeviceStatus::Ok);
    Same(staged, serial);
    if (fault < 10 || fault == 12) EXPECT_EQ(staged.storage.result.diagnostics.stiffness_rate_bound, -719.125);
    if (fault == 8 || fault == 12) EXPECT_EQ(staged.storage.control.node, staged.nodes[1].node);
    if (fault == 9) EXPECT_EQ(staged.storage.control.node, staged.nodes[0].node);
    serial.Reset(); staged.Reset();
    Serial(serial); Staged(staged);
    ASSERT_EQ(staged.storage.control.status, c::NodalWallDeviceStatus::Ok);
    Same(staged, serial);
  }
}
TEST(MappedWallResponseHost, ZeroStiffnessConsumesNoMassBodyRootOrPositionAndPriorFailureIsUntouched) {
  Packet serial, staged;
  for (auto* p : {&serial, &staged}) {
    for (auto& node : p->nodes) node.stiffness.value = node.stiffness.upper = -0.;
    std::fill(p->roots.begin(), p->roots.end(), p->groups+9);
    std::fill(p->inverse.begin(), p->inverse.end(), NAN);
    std::fill(p->position.begin(), p->position.end(), NAN);
    for (auto& body : p->bodies) body.mass = NAN;
    p->storage.model.config.owner.fixed_dt = -0.;
  }
  Serial(serial); Staged(staged);
  ASSERT_EQ(staged.storage.control.status, c::NodalWallDeviceStatus::Ok);
  Same(staged, serial);
  EXPECT_EQ(Bits(staged.summary.rate), Bits(0.));
  for (auto* p : {&serial, &staged}) {
    p->Reset();
    p->storage.control.status = c::NodalWallDeviceStatus::GeometryFailure;
    p->storage.control.node = 81;
    p->summary.parent_failure = 19;
    // Deliberately unavailable data after a prior stage failure.
    p->storage.result.nodes = nullptr;
    p->position.clear();
  }
  Serial(serial); Staged(staged);
  Same(staged, serial);
  EXPECT_EQ(staged.summary.rate, -37.25);
  EXPECT_EQ(staged.summary.parent_failure, 19u);
}
TEST(MappedWallResponseHost, ExactStepNeighborsAndOriginalNonpositiveDurationDomain) {
  Packet probe;
  Serial(probe);
  const auto threshold = 1.6/std::sqrt(probe.summary.rate);
  for (double dt : {0., -0., -1., std::nextafter(threshold, 0.), threshold,
      std::nextafter(threshold, INFINITY), 1e300}) {
    Packet serial, staged;
    serial.storage.model.config.owner.fixed_dt = staged.storage.model.config.owner.fixed_dt = dt;
    Serial(serial); Staged(staged);
    Same(staged, serial);
  }
}
} // namespace wall_response_test
