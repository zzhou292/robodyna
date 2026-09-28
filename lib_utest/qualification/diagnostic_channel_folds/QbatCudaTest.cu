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
} // namespace qbat_read_tile_test
