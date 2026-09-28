// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/qbat_measurement_read_tile/CudaTest.cu"
namespace qbat_read_tile_test {
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
    rows[63].internal_work[0]=poison;rows[64].internal_work[0]=poison;rows[128].internal_work[0]=-poison;rows[128].valid=0;
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
} // namespace qbat_read_tile_test
