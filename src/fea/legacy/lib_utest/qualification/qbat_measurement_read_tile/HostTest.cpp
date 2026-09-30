// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
#include <sys/mman.h>
#include <unistd.h>
#include <new>
namespace qbat_read_tile_test {
TEST(QbatMeasurementReadTileHost, InvalidPacketReadsOnlyItsWrittenValidityByte) {
  const auto page=std::size_t(sysconf(_SC_PAGESIZE));
  void* memory=mmap(nullptr,2*page,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
  ASSERT_NE(memory,MAP_FAILED);
  auto* source=new (static_cast<char*>(memory)+page-160) m::MeasurementParent;
  tile::Tile stage{};
  // Put valid/flags in the second page; every numerical payload is unreadable.
  ASSERT_EQ(mprotect(memory,page,PROT_NONE),0);
  for(std::uint8_t valid:{0u,2u,255u}) {
    source->valid=valid;tile::Read(stage,63,*source);
    EXPECT_EQ(stage.valid[63],valid);const auto value=tile::Load(stage,63);
    EXPECT_EQ(value.valid,valid);
  }
  EXPECT_EQ(mprotect(memory,page,PROT_READ|PROT_WRITE),0);
  source->~MeasurementParent();EXPECT_EQ(munmap(memory,2*page),0);
}
TEST(QbatMeasurementReadTileHost, ValidInactiveHistoryAndNonfiniteOperandsRetainExactBits) {
  f::Fixture fixture(129);ASSERT_FALSE(HasFailure());fixture.Stage();tile::Tile stage{};
  auto source=fixture.host->assembly.measurement[0];source.active=0;source.newly_removed=7;
  source.internal_work[0]=-0.;source.internal_work[1]=std::nan("31");
  source.kick_operand[3]=-std::numeric_limits<double>::infinity();
  source.drift_operand[2]=std::numeric_limits<double>::denorm_min();
  for(unsigned lane:{0u,31u,32u,63u}) {
    tile::Read(stage,lane,source);const auto copy=tile::Load(stage,lane);
    EXPECT_EQ(copy.valid,1);EXPECT_EQ(copy.active,0);EXPECT_EQ(copy.newly_removed,7);
    auto a=Seed(true,-0.),c=a;
    m::AccumulateMeasurementParent(fixture.host->model,source,0,a);
    m::AccumulateMeasurementParent(fixture.host->model,copy,0,c);
    EXPECT_TRUE(b::SameDiagnostics(a,c));
    EXPECT_EQ(f::Bits(copy.internal_work[0]),f::Bits(source.internal_work[0]));
    EXPECT_EQ(f::Bits(copy.internal_work[1]),f::Bits(source.internal_work[1]));
    EXPECT_EQ(f::Bits(copy.kick_operand[3]),f::Bits(source.kick_operand[3]));
  }
}
TEST(QbatMeasurementReadTileHost, ExtractedFoldAndPrescanKeepAllFailurePhases) {
  f::Fixture fixture(129);ASSERT_FALSE(HasFailure());
  for(unsigned fault=0;fault<=10;++fault)for(bool coupled:{false,true}) {
    fixture.Reset();fixture.host->model.config.usage=coupled?q::BatchUsage::CoupledForces:q::BatchUsage::PrescribedFields;
    f::Fault(fixture,fault);fixture.Stage();CompareHost(fixture,Seed(true,0x1p54));
  }
  fixture.Reset();fixture.Stage();fixture.host->candidate_status[128]=q::Status::kInvalidInput;
  const auto saved=fixture.host->assembly.measurement;fixture.host->assembly.measurement=nullptr;
  CompareHost(fixture,Seed(true,-0.));EXPECT_EQ(fixture.host->control.status,q::BatchStatus::ElementFailure);
  fixture.host->assembly.measurement=saved;
}
} // namespace qbat_read_tile_test
