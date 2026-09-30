// SPDX-License-Identifier: AGPL-3.0-or-later
// Reuse every existing complete-Control and failure-prefix assertion.
#include "lib_utest/qualification/solid_measurement_read_tile/CudaTest.cu"
namespace solid_read_tile_test {
inline double& ChannelOperand(b::MeasurementOperands<8>& row,unsigned c) {
  if(c==0)return row.work;if(c==1)return row.hourglass_work;if(c==2)return row.distortion_work;
  if(c==3)return row.plastic_work;if(c==4)return row.kick[7];return row.drift[7];
}
TEST(SolidChannelsCuda, EveryChannelFailureReplaysAllFailingParentWrites) {
  for(unsigned c=0;c<6;++c)for(unsigned fault:{0u,31u,63u,64u,65u,128u}) {
    SCOPED_TRACE(::testing::Message()<<c<<":"<<fault);Rig rig(129);ASSERT_FALSE(HasFailure());
    auto& family=rig.state->solid24;
    for(unsigned p=0;p<129;++p){family.measurement[p].distortion_work=(p%2?-.75:.5);family.measurement[p].native_dt=.01/(p+1);}
    ChannelOperand(family.measurement[fault],c)=std::nan("73");family.status[128]=19;
    rig.state->solid6z.status[0]=23;const auto result=rig.Compare();ASSERT_FALSE(HasFailure());
    EXPECT_EQ(result.parent,fault);EXPECT_EQ(result.family,s::Family::Solid24);
    EXPECT_EQ(result.status,fault==128?s::BatchStatus::ElementFailure:s::BatchStatus::NonfiniteResult);
  }
}
TEST(SolidChannelsCuda, OverflowBeforeLaterCancellationOrStatusUsesFirstPrefix) {
  for(unsigned c=0;c<6;++c)for(unsigned fault:{1u,63u,64u,65u,127u}) {
    Rig rig(129);ASSERT_FALSE(HasFailure());auto& family=rig.state->solid18_law90;
    ChannelOperand(family.measurement[fault-1],c)=std::numeric_limits<double>::max();
    ChannelOperand(family.measurement[fault],c)=std::numeric_limits<double>::max();
    ChannelOperand(family.measurement[128],c)=-std::numeric_limits<double>::infinity();
    const auto result=rig.Compare(false,Seed(true,0.));ASSERT_FALSE(HasFailure());EXPECT_EQ(result.parent,fault);
    family.status[128]=37;rig.Compare();ASSERT_FALSE(HasFailure());
  }
}
TEST(SolidChannelsCuda, EveryChannelHasCrossTileCancellationAndNativeMinimumTies) {
  Rig rig(129);ASSERT_FALSE(HasFailure());auto& family=rig.state->solid24;
  const double terms[]{0x1p54,1,-0x1p54};
  for(unsigned c=0;c<6;++c)for(unsigned p=0;p<129;++p)ChannelOperand(family.measurement[p],c)=0;
  for(unsigned c=0;c<6;++c)for(unsigned i=0;i<3;++i)ChannelOperand(family.measurement[63+i],c)=terms[i];
  family.measurement[63].native_dt=-0.;family.measurement[64].native_dt=0.;
  for(double seed:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}) {rig.Compare(false,Seed(true,seed));ASSERT_FALSE(HasFailure());}
}
} // namespace solid_read_tile_test
