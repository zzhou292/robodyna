// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
#include <sys/mman.h>
#include <unistd.h>
#include <new>
namespace solid_read_tile_test {
TEST(SolidMeasurementReadTileHost, StatusAndValidityShortCircuitUnavailableStorage) {
  tile::Tile stage{};b::DeviceFamily<b::Traits18> family;
  int failure=7;family.status=&failure;
  tile::Read(stage,63,family,0,true);EXPECT_EQ(stage.status[63],7);
  failure=0;std::uint8_t valid=0;family.result_valid=&valid;
  for(std::uint8_t value:{0u,2u,255u}) {valid=value;tile::Read(stage,63,family,0,true);EXPECT_EQ(stage.valid[63],value);}
}
TEST(SolidMeasurementReadTileHost, InitialSummaryDoesNotReadKickOrDriftPayload) {
  const auto page=std::size_t(sysconf(_SC_PAGESIZE));
  void* memory=mmap(nullptr,2*page,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);ASSERT_NE(memory,MAP_FAILED);
  auto* row=new (static_cast<char*>(memory)+page-4*sizeof(double)) b::MeasurementOperands<8>;
  row->work=-0.;row->hourglass_work=.125;row->plastic_work=std::nan("17");row->native_dt=.01;
  ASSERT_EQ(mprotect(static_cast<char*>(memory)+page,page,PROT_NONE),0);
  b::DeviceFamily<b::Traits18> family;int status=0;std::uint8_t valid=1;
  family.status=&status;family.result_valid=&valid;family.measurement=row;
  tile::Tile stage{};tile::Read(stage,0,family,0,false);const auto copy=tile::Load<b::Traits18>(stage,0,false);
  EXPECT_TRUE(std::signbit(copy.work));EXPECT_EQ(copy.hourglass_work,.125);EXPECT_TRUE(std::isnan(copy.plastic_work));
  EXPECT_EQ(mprotect(static_cast<char*>(memory)+page,page,PROT_READ|PROT_WRITE),0);
  row->~MeasurementOperands();EXPECT_EQ(munmap(memory,2*page),0);
}
template<class Traits> void CompareFold(unsigned index) {
  b::Storage state;Rows<Traits> rows(state,129);fe::NodalPreparedView view;
  for(unsigned fault=0;fault<4;++fault) {
    std::fill(rows.status.begin(),rows.status.end(),0);std::fill(rows.valid.begin(),rows.valid.end(),1);
    rows.values[63].work=fault==2?std::numeric_limits<double>::max():0x1p54;
    rows.values[64].work=fault==2?std::numeric_limits<double>::max():1;
    rows.values[128].work=-0x1p54;
    if(fault==1)rows.status[128]=11;if(fault==3)rows.valid[64]=0;
    b::Control expected;expected.diagnostics=Seed(true,-0.);auto current=expected;
    const auto result=b::read_tile_reference::MeasureOperandFamily<Traits>(state,expected,index,&view);
    EXPECT_EQ(b::MeasureOperandFamily<Traits>(state,current,index,&view),result);Same(current,expected);
  }
}
template<class Traits> void Transport() {
  b::Storage state;Rows<Traits> rows(state,2);tile::Tile stage{};
  for(unsigned p=0;p<2;++p) {
    auto& source=rows.values[p];source.work=p?std::nan("59"):-0.;source.native_dt=.125*(p+1);
    source.hourglass_work=-.25*(p+1);source.plastic_work=std::numeric_limits<double>::denorm_min();
    for(unsigned n=0;n<Traits::nodes;++n) {source.kick[n]=(p+1)*(.125+n);source.drift[n]=-(p+1)*(.25+n);}
    tile::Read(stage,63-p,b::FamilyStorage<Traits>(state),p,true);
    const auto value=tile::Load<Traits>(stage,63-p,true);
    using tl::fea::shell_startup_detail::SameBits;
    EXPECT_TRUE(SameBits(value.work,source.work));EXPECT_TRUE(SameBits(value.native_dt,source.native_dt));
    EXPECT_TRUE(SameBits(value.hourglass_work,source.hourglass_work));EXPECT_TRUE(SameBits(value.plastic_work,source.plastic_work));
    for(unsigned n=0;n<Traits::nodes;++n) {EXPECT_TRUE(SameBits(value.kick[n],source.kick[n]));EXPECT_TRUE(SameBits(value.drift[n],source.drift[n]));}
  }
}
TEST(SolidMeasurementReadTileHost, SixAndEightSlotTransportPreservesEveryOperandBit) {
  Transport<b::Traits18>();Transport<b::Traits6z>();
}
TEST(SolidMeasurementReadTileHost, ExtractedParentFoldKeepsFiveFamilyFailurePrefixes) {
  CompareFold<b::Traits18>(0);CompareFold<b::Traits24>(1);CompareFold<b::Traits6z>(2);
  CompareFold<b::Traits18Law44>(3);CompareFold<b::Traits18Law90>(4);
}
} // namespace solid_read_tile_test
