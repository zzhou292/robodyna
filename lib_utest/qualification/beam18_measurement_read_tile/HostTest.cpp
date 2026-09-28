// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
namespace beam18_read_tile_test {
TEST(Beam18ReadTileHost, RefactoredFoldPreservesFrozenParentFailureAndPartialDiagnostics) {
  b::Parent parent;parent.material_index=0;parent.reference=beam18_force_test::Reference();parent.domain_nodes[0]=0;parent.domain_nodes[1]=1;
  auto material=beam18_force_test::Material(parent.reference);b::ForceTrial value;
  ASSERT_EQ(b::InitializeForce(parent.reference,material,{},value),b::Status::Success);
  int status=0;d::Storage s;s.parents=&parent;s.materials=&material;s.slab[0]=s.slab[1]=&value;s.status=&status;s.count=1;
  for(unsigned fault=0;fault<4;++fault) {
    auto copy=value;status=0;if(fault==1)status=int(b::Status::InvalidInput);if(fault==2)copy.geometry.length_m=-1;if(fault==3)copy.diagnostics.internal_work_increment_j[0]=std::numeric_limits<double>::max();
    s.slab[0]=s.slab[1]=&copy;s.control=Poison();s.control.diagnostics=Seed(false,std::numeric_limits<double>::max());
    const auto seed=s.control;const bool expected=d::read_tile_reference::Measure(s,0,1,nullptr);const auto old=s.control;s.control=seed;
    EXPECT_EQ(d::Measure(s,0,1,nullptr),expected);Same(s.control,old);
  }
}
TEST(Beam18ReadTileHost, TileAndRetainedArenaRemainBounded) {EXPECT_EQ(sizeof(tile::Tile),2376u);EXPECT_EQ(sizeof(d::MeasurementOperands),32u);}
}
