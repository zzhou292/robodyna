// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/qbat_measurement_read_tile/Support.h"
#include "lib_src/elements/qbat/mapped/measurement/Channels.h"
namespace qbat_read_tile_test {
bool Channels(f::Fixture& fixture,q::BatchDiagnostics& d) {
  tile::Tile stage{};const auto& model=fixture.host->model;d.element_count=model.config.element_count;
  for(std::size_t first=0;first<model.config.element_count;first+=tile::Threads) {
    unsigned count=unsigned(std::min(std::size_t(tile::Threads),model.config.element_count-first));bool serial=false;
    for(unsigned p=0;p<count;++p){tile::Read(stage,p,fixture.host->assembly.measurement[first+p]);serial=serial||stage.valid[p]!=1;}
    tile::SeedChannels(stage,d);
    if(serial){if(!tile::ReplayTile(model,stage,first,count,d))return false;continue;}
    for(unsigned c=0;c<8;++c)tile::ScalarChannel(stage,c,count);
    for(unsigned c=8;c<10;++c)tile::WorkChannel(stage,c,count,model.config.usage==q::BatchUsage::CoupledForces);
    for(unsigned c=0;c<3;++c)tile::MinimumChannel(stage,c,first,count);
    tile::MaximumChannel(stage,count);tile::CountChannel(stage,false,count);tile::CountChannel(stage,true,count);
    tile::StoreChannels(stage,d);
  }
  return true;
}
void CompareChannels(f::Fixture& fixture,double seed) {
  auto expected=Seed(true,seed),current=expected;
  EXPECT_EQ(Channels(fixture,current),m::MeasureStagedParents(fixture.host->model,fixture.host->assembly.measurement,expected));
  EXPECT_TRUE(b::SameDiagnostics(current,expected));
}
TEST(QbatChannelsHost, ScalarLedgerCountsExtremaAndSignedZeroAreExact) {
  for(unsigned count:{1u,63u,64u,65u,129u})for(bool coupled:{false,true}) {
    f::Fixture fixture(count);ASSERT_FALSE(HasFailure());fixture.Stage();
    fixture.host->model.config.usage=coupled?q::BatchUsage::CoupledForces:q::BatchUsage::PrescribedFields;
    for(unsigned p=0;p<count;++p){auto& v=fixture.host->assembly.measurement[p];v.active=p%2?255:0;v.newly_removed=p%2?0:255;}
    CompareChannels(fixture,-0.);
  }
}
TEST(QbatChannelsHost, DerivedOverflowRemainsDelayedPastEveryAdmittedParent) {
  f::Fixture fixture(129);ASSERT_FALSE(HasFailure());fixture.Stage();
  fixture.host->assembly.measurement[0].internal_work[0]=std::numeric_limits<double>::max();
  fixture.host->assembly.measurement[1].internal_work[0]=std::numeric_limits<double>::max();
  fixture.host->assembly.measurement[128].internal_work[0]=-std::numeric_limits<double>::infinity();
  CompareChannels(fixture,.25);
  fixture.host->assembly.measurement[128].valid=0;CompareChannels(fixture,.25);
}
TEST(QbatChannelsHost, EveryInvalidTileRetainsExactPartialCountsAndSums) {
  for(unsigned p:{0u,31u,63u,64u,65u,128u}) {
    f::Fixture fixture(129);ASSERT_FALSE(HasFailure());fixture.Stage();fixture.host->assembly.measurement[p].valid=255;
    CompareChannels(fixture,0x1p54);
  }
}
TEST(QbatChannelsHost, OnlyTransientTileStorageGrows) {
  EXPECT_EQ(sizeof(tile::Tile),10576u);EXPECT_EQ(sizeof(m::MeasurementParent),168u);EXPECT_EQ(sizeof(b::Storage),680u);
}
} // namespace qbat_read_tile_test
