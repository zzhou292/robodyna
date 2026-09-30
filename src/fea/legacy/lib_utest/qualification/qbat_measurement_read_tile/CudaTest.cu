// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
#include "lib_utest/qualification/qbat_measurement_operands/DeviceFixture.cuh"
#include "lib_src/elements/qbat/mapped/measurement/Finalize.cuh"
namespace qbat_read_tile_test {
__global__ void FrozenLeaf(b::Storage* state,tl::fea::NodalPreparedView view,q::BatchDiagnostics seed) {
  m::read_tile_reference::FinalizeMeasurement(*state,view,seed,1);
}
__global__ void CurrentLeaf(b::Storage* state,tl::fea::NodalPreparedView view,q::BatchDiagnostics seed) {
  __shared__ tile::Tile stage;tile::Finalize(*state,view,seed,1,stage);
}
void Drain() {ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);}
b::Control Compare(f::DeviceFixture& rig,const q::BatchDiagnostics& seed) {
  const auto view=rig.input->Prepared(2);rig.storage->control=Poison();
  FrozenLeaf<<<1,1>>>(rig.storage,view,seed);Drain();const auto expected=rig.storage->control;
  rig.storage->control=Poison();CurrentLeaf<<<1,tile::Threads>>>(rig.storage,view,seed);Drain();
  Same(rig.storage->control,expected);return rig.storage->control;
}
TEST(QbatMeasurementReadTileCuda, PartialTilesEmptyInputAndInactiveHistories) {
  for(std::size_t count:{1u,2u,31u,32u,33u,63u,64u,65u,127u,128u,129u,193u}) {
    SCOPED_TRACE(count);f::DeviceFixture rig(count);ASSERT_FALSE(HasFailure());rig.source.Stage();rig.Restore();
    for(std::size_t parent=0;parent<count;++parent) {
      auto& row=rig.storage->assembly.measurement[parent];row.active=parent%3!=0;
      row.newly_removed=parent%3==0;row.internal_work[0]=parent%4?-.25:.5;
    }
    for(bool coupled:{false,true}) {
      rig.storage->model.config.usage=coupled?q::BatchUsage::CoupledForces:q::BatchUsage::PrescribedFields;
      Compare(rig,Seed(true,-0.));ASSERT_FALSE(HasFailure());
    }
  }
  f::DeviceFixture rig(1);ASSERT_FALSE(HasFailure());rig.source.Stage();rig.Restore();
  rig.storage->model.config.element_count=0;rig.storage->candidate_status=nullptr;
  rig.storage->assembly.measurement=nullptr;Compare(rig,Seed(false,.25));ASSERT_FALSE(HasFailure());
}
TEST(QbatMeasurementReadTileCuda, FullStatusPrescanDominatesMeasurementAndRepairsSameArena) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());rig.source.Stage();rig.Restore();
  rig.storage->candidate_status[128]=q::Status::kInvalidInput;
  rig.storage->candidate_status[64]=q::Status::kInvalidReference;
  const auto rows=rig.storage->assembly.measurement;const auto maximum=rig.storage->assembly.maximum;
  rig.storage->assembly.measurement=nullptr;rig.storage->assembly.maximum=nullptr;
  const auto failed=Compare(rig,Seed(true,std::nan("31")));ASSERT_FALSE(HasFailure());
  EXPECT_EQ(failed.status,q::BatchStatus::ElementFailure);EXPECT_EQ(failed.element,64u);
  rig.storage->assembly.measurement=rows;rig.storage->assembly.maximum=maximum;
  rig.storage->candidate_status[64]=rig.storage->candidate_status[128]=q::Status::kSuccess;
  CurrentLeaf<<<1,tile::Threads>>>(rig.storage,rig.input->Prepared(2),Seed(false,.25));Drain();
  const auto repaired=rig.storage->control;EXPECT_EQ(repaired.status,q::BatchStatus::Success);
  FrozenLeaf<<<1,1>>>(rig.storage,rig.input->Prepared(2),Seed(false,.25));Drain();Same(repaired,rig.storage->control);
}
TEST(QbatMeasurementReadTileCuda, InvalidPacketRetainsExactSourcePrefixAndSkipsMaximum) {
  f::DeviceFixture rig(193);ASSERT_FALSE(HasFailure());
  for(std::size_t fault:{0u,31u,32u,63u,64u,65u,127u,128u,192u})for(std::uint8_t valid:{0u,2u,255u}) {
    SCOPED_TRACE(::testing::Message()<<fault<<":"<<unsigned(valid));rig.source.Stage();rig.Restore();
    auto& bad=rig.storage->assembly.measurement[fault];bad.valid=valid;
    bad.internal_work[0]=bad.kick_operand[0]=std::nan("41");
    rig.storage->assembly.maximum=nullptr;
    const auto failed=Compare(rig,Seed(true,.25));ASSERT_FALSE(HasFailure());
    EXPECT_EQ(failed.status,q::BatchStatus::NonfiniteResult);
    EXPECT_EQ(failed.diagnostics.active_count,7u+fault);
  }
}
TEST(QbatMeasurementReadTileCuda, CrossTileCancellationKeepsParentLocalChannelOrder) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(unsigned first:{62u,63u,126u})for(double seed:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}) {
    rig.source.Stage();rig.Restore();
    for(unsigned i=0;i<129;++i) {
      auto& row=rig.storage->assembly.measurement[i];
      for(unsigned c=0;c<2;++c)row.internal_work[c]=row.internal_increment[c]=0;
      for(unsigned local=0;local<4;++local)row.kick_operand[local]=row.drift_operand[local]=0;
    }
    const double terms[]{0x1p54,1,-0x1p54};
    for(unsigned i=0;i<3;++i) {
      auto& row=rig.storage->assembly.measurement[first+i];
      for(unsigned c=0;c<2;++c)row.internal_work[c]=row.internal_increment[c]=terms[i];
      row.kick_operand[0]=terms[i];row.drift_operand[3]=terms[i];
    }
    Compare(rig,Seed(true,seed));ASSERT_FALSE(HasFailure());
  }
}
TEST(QbatMeasurementReadTileCuda, OverflowNaNAndDisplacementFallbackPreserveControl) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(unsigned fault=0;fault<=10;++fault)for(bool valid:{false,true}) {
    rig.source.Reset();f::Fault(rig.source,fault);rig.source.Stage();rig.Restore();
    Compare(rig,Seed(valid,0x1p54));ASSERT_FALSE(HasFailure());
  }
  for(double poison:{std::numeric_limits<double>::max(),double(INFINITY),std::nan("47")}) {
    rig.source.Reset();rig.source.Stage();rig.Restore();
    rig.storage->assembly.measurement[63].internal_work[0]=poison;
    rig.storage->assembly.measurement[64].internal_work[0]=poison;
    rig.storage->assembly.measurement[128].internal_work[0]=-poison;
    Compare(rig,Seed(true,0.));ASSERT_FALSE(HasFailure());
    rig.storage->assembly.measurement[128].valid=0;Compare(rig,Seed(false,0.));ASSERT_FALSE(HasFailure());
  }
}
TEST(QbatMeasurementReadTileCuda, ProductionMappedCallerMatchesIndependentFullSerialCaller) {
  f::DeviceFixture actual(129),expected(129);ASSERT_FALSE(HasFailure());
  for(unsigned fault=0;fault<=10;++fault)for(bool valid:{false,true}) {
    for(auto* p:{&actual,&expected}) {p->source.Reset();f::Fault(p->source,fault);p->Restore();p->storage->control=Poison();}
    const auto seed=Seed(valid,0x1p54);
    b::LaunchMappedMeasurements(actual.storage,&actual.storage->slab[0],&actual.storage->slab[1],actual.input->Prepared(2),seed,f::g::Nodes);
    f::g::serial_candidate::FinalizeCandidate<<<1,1>>>(expected.storage,&expected.storage->slab[0],&expected.storage->slab[1],expected.input->Prepared(2),seed);
    Drain();ASSERT_FALSE(HasFailure());f::Same(actual,expected);
  }
}
} // namespace qbat_read_tile_test
