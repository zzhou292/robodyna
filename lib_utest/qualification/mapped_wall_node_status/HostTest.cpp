#include "lib_src/collision/nodal_wall_mapped/NodeStatus.h"
#include <gtest/gtest.h>

namespace wall_node_status_test {
namespace m = tlfea::contact::nodal_wall_mapped;
namespace d = tlfea::contact::nodal_wall_device_detail;
using Code = tlfea::contact::NodalWallDeviceStatus;

struct StatusFixture {
  d::Storage storage;
  d::Control status[5];
  StatusFixture() {
    storage.model.node_count = 5;
    storage.node_status = status;
    status[1].status = Code::GeometryFailure;
    status[1].node = 900; // Earlier compact slot, larger global NID.
    status[1].parent = 71;
    status[1].point.cause = static_cast<decltype(status[1].point.cause)>(3);
    status[4].status = Code::AssemblyFailure;
    status[4].node = 2;
  }
};

TEST(WallNodeStatusHost, CompactWinnerCopiesCompleteFailureAndPreservesRows) {
  StatusFixture fixture;
  const auto before = fixture.status[1];
  m::CopyNodeFailure(fixture.storage, 1);
  EXPECT_EQ(fixture.storage.control.status, before.status);
  EXPECT_EQ(fixture.storage.control.node, 900u);
  EXPECT_EQ(fixture.storage.control.parent, 71u);
  EXPECT_EQ(fixture.storage.control.point.cause, before.point.cause);
  EXPECT_EQ(fixture.status[4].node, 2u);
}

TEST(WallNodeStatusHost, MalformedOrSuccessfulIndexedSlotFallsBackToOriginalOrder) {
  for (auto key : {0ull, 5ull, (1ull << 40)}) {
    StatusFixture fixture;
    m::CopyNodeFailure(fixture.storage, key);
    EXPECT_EQ(fixture.storage.control.status, Code::GeometryFailure);
    EXPECT_EQ(fixture.storage.control.node, 900u);
    EXPECT_EQ(fixture.storage.control.parent, 71u);
  }
}

TEST(WallNodeStatusHost, NoFailureAndEmptyFallbackPreserveEarlierControl) {
  StatusFixture fixture;
  fixture.storage.control.status = Code::InvalidMass;
  fixture.storage.control.node = 42;
  m::CopyNodeFailure(fixture.storage, ~0ull);
  EXPECT_EQ(fixture.storage.control.status, Code::InvalidMass);
  EXPECT_EQ(fixture.storage.control.node, 42u);
  for (auto& status : fixture.status) status = {};
  m::CopyNodeFailure(fixture.storage, 5);
  EXPECT_EQ(fixture.storage.control.status, Code::InvalidMass);
  EXPECT_EQ(fixture.storage.control.node, 42u);
}

TEST(WallNodeStatusHost, PointAdmissionCanReplaceEarlierControlAsOriginalScanDid) {
  StatusFixture fixture;
  fixture.storage.control.status = Code::InvalidMass;
  m::CopyNodeFailure(fixture.storage, 1);
  EXPECT_EQ(fixture.storage.control.status, Code::GeometryFailure);
  EXPECT_EQ(fixture.storage.control.node, 900u);
}
} // namespace wall_node_status_test
