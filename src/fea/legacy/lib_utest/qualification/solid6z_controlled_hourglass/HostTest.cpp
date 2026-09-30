// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace s6_control_test {
TEST(S6Controlled, NativeProjectionAllHistorySlotsFoldAndAssembly){for(const auto& x:Cases())Check(x);}
TEST(S6Controlled, NativeConstructorAndUniformTranslation){auto x=Base();x.initialization=true;x.interval.dt_s=0;for(auto& v:x.interval.velocity_midpoint_m_s)v={1,-2,.5};Check(x);}
TEST(S6Controlled, IndependentLoadingUnloadingCarry32Intervals){auto x=Base();NativeHistory native(x);
 for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);x.interval=legacy::Path(x.reference,step*12);Trial r;ASSERT_EQ(Evaluate(x,r),s::Status::Success);const auto n=native.Step(x);Compare(x,r,n);ASSERT_FALSE(HasFailure());x.accepted=r.result.proposed_values;native.accepted=n.history;}
}
TEST(S6Controlled, FourthModeAndNonzeroIncomingMaterialForceRemainNative){auto x=Base();x.interval=legacy::Path(x.reference,33);auto r=Check(x);bool material=false;
 for(const auto& f:r.result.material_local_force_n)material|=f.x!=0||f.y!=0||f.z!=0;EXPECT_TRUE(material);EXPECT_NE(r.result.expanded_hourglass.work_j,0);
 for(unsigned k=0;k<3;++k){EXPECT_EQ(b::Component(r.result.local_force_n[2],k),b::Component(r.result.expanded_hourglass.local_force_n[2],k)+b::Component(r.result.expanded_hourglass.local_force_n[3],k));EXPECT_EQ(b::Component(r.result.local_force_n[5],k),b::Component(r.result.expanded_hourglass.local_force_n[6],k)+b::Component(r.result.expanded_hourglass.local_force_n[7],k));}
}
TEST(S6Controlled, InvalidAndLateOverflowPreserveOutputAndAcceptedState){auto x=Base();x.interval=legacy::Path(x.reference,10);Trial r=Check(x);const auto before=Values(r);
 auto bad=x;bad.accepted.controlled_hourglass.force_n[1][2]=std::numeric_limits<double>::max();EXPECT_NE(Evaluate(bad,r),s::Status::Success);EXPECT_EQ(Values(r),before);
 bad=x;bad.interval.velocity_midpoint_m_s[0].x=std::numeric_limits<double>::infinity();EXPECT_NE(Evaluate(bad,r),s::Status::Success);EXPECT_EQ(Values(r),before);
 EXPECT_EQ(Evaluate(x,r),s::Status::Success);EXPECT_EQ(Values(r),before);
}
} // namespace s6_control_test
