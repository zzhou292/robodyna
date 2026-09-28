// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
namespace type13_read_tile_test {
TEST(Type13ReadTileHost, RefactoredFoldRetainsLiteralFrozenArithmetic) {
  for(std::size_t n:{1u,63u,64u,65u,129u,4442u}) for(double seed:{0.,-0.,0x1p54}) {
    std::vector<b::Measurement> rows(n);for(std::size_t i=0;i<n;++i)rows[i]=Row(i);
    b::DeviceModel model;model.element_count=n;auto old=Poison(),now=Poison();old.diagnostics=now.diagnostics=Seed(true,seed);
    const bool a=b::read_tile_reference::MeasurePrepared(model,rows.data(),old.diagnostics);
    const bool z=b::MeasurePrepared(model,rows.data(),now.diagnostics);EXPECT_EQ(a,z);Same(now,old);
  }
}
TEST(Type13ReadTileHost, SharedTileAddsNoRetainedStorageAndKeepsRawFlagValues) {
  EXPECT_EQ(sizeof(tile::Tile),9096u);EXPECT_EQ(sizeof(b::Measurement),144u);
  tile::Tile stage;auto row=Row(3);row.active=255;row.newly_failed=2;row.work[0]=-0.;row.increment[0]=std::nan("7");
  tile::Read(stage,63,row);const auto loaded=tile::Load(stage,63);
  b::DeviceModel model;model.element_count=1;auto a=Poison(),z=Poison();a.diagnostics=z.diagnostics=Seed();
  b::MeasurePrepared(model,&row,a.diagnostics);b::MeasurePrepared(model,&loaded,z.diagnostics);Same(a,z);
}
}
