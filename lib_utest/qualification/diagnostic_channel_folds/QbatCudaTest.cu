// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/qbat_measurement_read_tile/CudaTest.cu"
#include "BaselineFinalize.cuh"
#include <cstdio>
namespace qbat_read_tile_test {
__global__ void BaselineLeaf(b::Storage* state,tl::fea::NodalPreparedView view,q::BatchDiagnostics seed) {
  __shared__ m::baseline_measurement::Tile stage;m::baseline_measurement::Finalize(*state,view,seed,1,stage);
}
// The literal qualified kernel owns regression bits for new NaN arithmetic
// cases. The unchanged older serial oracle remains in every original test.
b::Control CompareBaseline(f::DeviceFixture& rig,const q::BatchDiagnostics& seed) {
  const auto view=rig.input->Prepared(2);
  rig.storage->control=Poison();BaselineLeaf<<<1,64>>>(rig.storage,view,seed);Drain();const auto expected=rig.storage->control;
  rig.storage->control=Poison();CurrentLeaf<<<1,tile::Threads>>>(rig.storage,view,seed);Drain();Same(rig.storage->control,expected);
  EXPECT_EQ(rig.storage->control.status,q::BatchStatus::NonfiniteResult);return rig.storage->control;
}
inline double& ChannelOperand(m::MeasurementParent& row,unsigned c) {
  if(c<2)return row.internal_work[c];if(c<4)return row.internal_increment[c-2];
  if(c==4)return row.plastic_work;if(c==5)return row.plastic_increment;
  if(c==6)return row.viscous_work;if(c==7)return row.viscous_increment;
  if(c==8)return row.kick_operand[3];return row.drift_operand[3];
}
TEST(QbatChannelsCuda, EveryLedgerKeepsDelayedOverflowAndCompleteDiagnostics) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(unsigned c=0;c<10;++c) {
    rig.source.Reset();rig.source.Stage();rig.Restore();auto* rows=rig.storage->assembly.measurement;
    ChannelOperand(rows[63],c)=std::numeric_limits<double>::max();
    ChannelOperand(rows[64],c)=std::numeric_limits<double>::max();ChannelOperand(rows[128],c)=-INFINITY;
    Compare(rig,Seed(true,0.));ASSERT_FALSE(HasFailure());
    rows[128].valid=0;Compare(rig,Seed(true,0.));ASSERT_FALSE(HasFailure());
  }
}
TEST(QbatChannelsCuda, EveryLedgerCancellationAndExtremaFirstParentRules) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());rig.source.Stage();rig.Restore();auto* rows=rig.storage->assembly.measurement;
  const double terms[]{0x1p54,1,-0x1p54};
  for(unsigned c=0;c<10;++c)for(unsigned p=0;p<129;++p)ChannelOperand(rows[p],c)=0;
  for(unsigned c=0;c<10;++c)for(unsigned i=0;i<3;++i)ChannelOperand(rows[63+i],c)=terms[i];
  rows[0].area_ratio=-0.;rows[1].area_ratio=0.;rows[0].thickness_ratio=std::nan("79");rows[64].native_dt=std::nan("83");
  rows[0].newly_removed=255;rows[1].active=255;
  for(double seed:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}) {Compare(rig,Seed(true,seed));ASSERT_FALSE(HasFailure());}
}
TEST(QbatChannelsCuda, NonfinitePrefixReportsEachNumericField) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(double poison:{std::numeric_limits<double>::max(),double(INFINITY),std::nan("47")}) {
    SCOPED_TRACE(::testing::Message()<<"poison bits="<<f::Bits(poison));
    rig.source.Reset();rig.source.Stage();rig.Restore();auto* rows=rig.storage->assembly.measurement;
    rows[63].internal_work[0]=poison;rows[64].internal_work[0]=poison;rows[128].internal_work[0]=-poison;
    const auto view=rig.input->Prepared(2);const auto seed=Seed(false,0.);
    FrozenLeaf<<<1,1>>>(rig.storage,view,seed);Drain();const auto expected=rig.storage->control;
    CurrentLeaf<<<1,tile::Threads>>>(rig.storage,view,seed);Drain();const auto actual=rig.storage->control;
    const auto& a=actual.diagnostics;const auto& e=expected.diagnostics;
    auto equal=[](const char* name,double x,double y){EXPECT_EQ(f::Bits(x),f::Bits(y))<<name;};
    for(unsigned c=0;c<2;++c){equal("internal_work",a.internal_work_j[c],e.internal_work_j[c]);equal("internal_increment",a.internal_work_increment_j[c],e.internal_work_increment_j[c]);}
    equal("plastic",a.plastic_work_j,e.plastic_work_j);equal("plastic_increment",a.plastic_work_increment_j,e.plastic_work_increment_j);
    equal("viscous",a.numerical_viscous_work_j,e.numerical_viscous_work_j);equal("viscous_increment",a.numerical_viscous_work_increment_j,e.numerical_viscous_work_increment_j);
    equal("kick",a.internal_kick_work,e.internal_kick_work);equal("drift",a.internal_drift_work,e.internal_drift_work);
    equal("area",a.minimum_area_ratio,e.minimum_area_ratio);equal("thickness",a.minimum_thickness_ratio,e.minimum_thickness_ratio);
    equal("dt",a.minimum_native_dt,e.minimum_native_dt);equal("displacement",a.maximum_displacement,e.maximum_displacement);equal("strain",a.maximum_absolute_strain,e.maximum_absolute_strain);
    EXPECT_EQ(a.active_count,e.active_count);EXPECT_EQ(a.newly_removed_count,e.newly_removed_count);Same(actual,expected);
  }
}
TEST(QbatChannelsCuda, SignedOpposedNaNPayloadsKeepTheSerialOperandSelection) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(unsigned c=0;c<10;++c)for(double first:{std::nan("47"),-std::nan("47"),double(INFINITY),double(-INFINITY)}) {
    rig.source.Reset();rig.source.Stage();rig.Restore();auto* rows=rig.storage->assembly.measurement;
    ChannelOperand(rows[63],c)=first;ChannelOperand(rows[64],c)=std::nan("53");ChannelOperand(rows[128],c)=-first;
    CompareBaseline(rig,Seed(true,0.));ASSERT_FALSE(HasFailure());
    rows[128].valid=0;CompareBaseline(rig,Seed(false,0.));ASSERT_FALSE(HasFailure());
  }
}
inline double& DiagnosticChannel(q::BatchDiagnostics& d,unsigned c) {
  if(c<2)return d.internal_work_j[c];if(c<4)return d.internal_work_increment_j[c-2];
  if(c==4)return d.plastic_work_j;if(c==5)return d.plastic_work_increment_j;
  if(c==6)return d.numerical_viscous_work_j;if(c==7)return d.numerical_viscous_work_increment_j;
  if(c==8)return d.internal_kick_work;return d.internal_drift_work;
}
TEST(QbatChannelsCuda, NonfiniteIncomingSeedsUseTheOriginalScalarAuthority) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(unsigned c=0;c<10;++c)for(double incoming:{std::nan("17"),-std::nan("17"),double(INFINITY),double(-INFINITY)}) {
    rig.source.Reset();rig.source.Stage();rig.Restore();auto* rows=rig.storage->assembly.measurement;
    auto seed=Seed(true,0.);DiagnosticChannel(seed,c)=incoming;
    seed.minimum_area_ratio=std::nan("23");seed.maximum_absolute_strain=-std::nan("29");
    ChannelOperand(rows[0],c)=-std::nan("31");ChannelOperand(rows[64],c)=std::nan("37");
    CompareBaseline(rig,seed);ASSERT_FALSE(HasFailure());rows[128].valid=0;CompareBaseline(rig,seed);ASSERT_FALSE(HasFailure());
  }
}
TEST(QbatChannelsCuda, Literal7695BaselineComparesBothNonfiniteOracleContexts) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  std::printf("kind,channel,input_bits,current_bits,baseline_bits,serial_bits,current_baseline_equal,baseline_serial_equal\n");
  for(unsigned kind=0;kind<2;++kind)for(unsigned c=0;c<10;++c)
  for(double input:{std::nan("17"),-std::nan("17"),double(INFINITY),double(-INFINITY)}) {
    rig.source.Reset();rig.source.Stage();rig.Restore();auto* rows=rig.storage->assembly.measurement;auto seed=Seed(true,0.);
    if(kind==0) {
      ChannelOperand(rows[63],c)=input;ChannelOperand(rows[64],c)=std::nan("53");ChannelOperand(rows[128],c)=-input;
    } else {
      DiagnosticChannel(seed,c)=input;seed.minimum_area_ratio=std::nan("23");seed.maximum_absolute_strain=-std::nan("29");
      ChannelOperand(rows[0],c)=-std::nan("31");ChannelOperand(rows[64],c)=std::nan("37");
    }
    const auto view=rig.input->Prepared(2);
    FrozenLeaf<<<1,1>>>(rig.storage,view,seed);Drain();auto serial=rig.storage->control;
    BaselineLeaf<<<1,64>>>(rig.storage,view,seed);Drain();auto baseline=rig.storage->control;
    CurrentLeaf<<<1,tile::Threads>>>(rig.storage,view,seed);Drain();auto current=rig.storage->control;
    std::printf("%u,%u,%016llx,%016llx,%016llx,%016llx,%d,%d\n",kind,c,
      (unsigned long long)f::Bits(input),(unsigned long long)f::Bits(DiagnosticChannel(current.diagnostics,c)),
      (unsigned long long)f::Bits(DiagnosticChannel(baseline.diagnostics,c)),(unsigned long long)f::Bits(DiagnosticChannel(serial.diagnostics,c)),
      int(b::SameDiagnostics(current.diagnostics,baseline.diagnostics)),int(b::SameDiagnostics(baseline.diagnostics,serial.diagnostics)));
    SCOPED_TRACE(::testing::Message()<<kind<<":"<<c<<":"<<f::Bits(input));Same(current,baseline);
  }
}
} // namespace qbat_read_tile_test
