// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/solid_measurement_read_tile/Support.h"
#include "lib_src/elements/solids/resident/measurement/Channels.h"
namespace solid_read_tile_test {
template<class Traits> bool Channels(b::Storage& state,b::Control& control,unsigned family,bool work) {
  tile::Tile stage{};auto& rows=b::FamilyStorage<Traits>(state);fe::NodalPreparedView view;
  control.diagnostics.parent_count[family]=rows.count;
  for(std::size_t first=0;first<rows.count;first+=tile::Threads) {
    unsigned count=unsigned(std::min(std::size_t(tile::Threads),rows.count-first));bool serial=false;
    for(unsigned p=0;p<count;++p){tile::Read(stage,p,rows,first+p,work);serial=serial||stage.status[p]!=0||stage.valid[p]!=1;}
    tile::SeedChannels(stage,control.diagnostics,family);
    if(!serial) {
      for(unsigned c=0;c<4;++c)tile::ScalarChannel(stage,c,count);
      for(unsigned c=4;c<6;++c)tile::WorkChannel<Traits::nodes>(stage,c,count,work);
      tile::MinimumChannel(stage,count);
    }
    if(serial||!tile::ChannelsFinite(stage)) {
      if(!tile::ReplayTile<Traits>(control,family,first,count,work?&view:nullptr,stage))return false;
    } else tile::StoreChannels(stage,control.diagnostics,family);
  }
  return true;
}
template<class Traits> void CompareChannels(unsigned family) {
  for(unsigned count:{0u,1u,63u,64u,65u,129u})for(bool work:{false,true}) {
    b::Storage state;Rows<Traits> rows(state,count);fe::NodalPreparedView view;
    for(unsigned p=0;p<count;++p)rows.values[p].distortion_work=(p%2?-.75:.5);
    b::Control expected;expected.diagnostics=Seed(true,-0.);auto current=expected;
    EXPECT_EQ(Channels<Traits>(state,current,family,work),b::MeasureOperandFamily<Traits>(state,expected,family,work?&view:nullptr));Same(current,expected);
  }
}
TEST(SolidChannelsHost, EveryFamilyChannelPreservesCompleteControlBits) {
  CompareChannels<b::Traits18>(0);CompareChannels<b::Traits24>(1);CompareChannels<b::Traits6z>(2);
  CompareChannels<b::Traits18Law44>(3);CompareChannels<b::Traits18Law90>(4);
}
inline double& Operand(b::MeasurementOperands<8>& row,unsigned channel) {
  if(channel==0)return row.work;if(channel==1)return row.hourglass_work;if(channel==2)return row.distortion_work;
  if(channel==3)return row.plastic_work;if(channel==4)return row.kick[7];return row.drift[7];
}
TEST(SolidChannelsHost, EveryFiniteChannelReplaysExactFirstFailingParent) {
  for(unsigned c=0;c<6;++c)for(unsigned fault:{0u,31u,63u,64u,65u,128u}) {
    b::Storage state;Rows<b::Traits24> rows(state,129);fe::NodalPreparedView view;
    Operand(rows.values[fault],c)=std::nan("73");rows.status[128]=19;
    b::Control expected;expected.diagnostics=Seed(true,.25);auto current=expected;
    EXPECT_EQ(Channels<b::Traits24>(state,current,1,true),b::MeasureOperandFamily<b::Traits24>(state,expected,1,&view));Same(current,expected);
  }
}
TEST(SolidChannelsHost, CancellationAndSkippedInitialWorkRemainOrdered) {
  for(double seed:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()})for(bool work:{false,true}) {
    b::Storage state;Rows<b::Traits24> rows(state,129);fe::NodalPreparedView view;
    const double terms[]{0x1p54,1,-0x1p54};
    for(unsigned c=0;c<6;++c)for(unsigned i=0;i<3;++i)Operand(rows.values[63+i],c)=terms[i];
    if(!work)rows.values[0].kick[0]=rows.values[0].drift[0]=std::nan("79");
    b::Control expected;expected.diagnostics=Seed(true,seed);auto current=expected;
    EXPECT_EQ(Channels<b::Traits24>(state,current,1,work),b::MeasureOperandFamily<b::Traits24>(state,expected,1,work?&view:nullptr));Same(current,expected);
  }
}
TEST(SolidChannelsHost, OnlyTransientTileStorageGrows) {
  EXPECT_EQ(sizeof(tile::Tile),11144u);EXPECT_EQ(sizeof(b::MeasurementOperands<8>),168u);
  EXPECT_EQ(sizeof(b::Storage),1360u);
}
} // namespace solid_read_tile_test
