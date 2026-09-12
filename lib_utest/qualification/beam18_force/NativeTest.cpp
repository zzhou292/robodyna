// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#ifdef BEAM18_FORCE_ORIGINAL
#include "OriginalFixture.h"
#endif
namespace beam18_force_test {
void Trajectory(const b::Reference& ref,unsigned steps) {
  const auto material=Material(ref);b::ForceTrial accepted,next;
  ASSERT_EQ(b::InitializeForce(ref,material,{11.123,-.37,.129},accepted),b::Status::Success);
  b::PrescribedInterval initial;
  for(unsigned n=0;n<2;++n) {initial.position_endpoint_m[n]=ref.geometry().endpoint_m[n];initial.velocity_midpoint_m_s[n]={11.123,-.37,.129};}
  auto native=Native(ref,material,{},initial,true);Compare(accepted,native);ASSERT_FALSE(::testing::Test::HasFailure());
  for(unsigned step=0;step<steps;++step) {
    SCOPED_TRACE(step);
    auto in=Motion(ref,accepted.proposed_history);
    const double sign=step%12<7?1.:-1.;
    const auto& g=accepted.geometry;
    in.dt_s=2e-6;
    const auto axial=tl::math::fixed3::Scale(g.axis[0],sign*(step%3?15.:.05));
    const auto shear=tl::math::fixed3::Scale(g.axis[1],sign*.8);
    in.velocity_midpoint_m_s[0]={11.123,-.37,.129};
    in.velocity_midpoint_m_s[1]=tl::math::fixed3::Add(in.velocity_midpoint_m_s[0],tl::math::fixed3::Add(axial,shear));
    in.angular_velocity_midpoint_rad_s[0]={12.,-7.,9.};
    in.angular_velocity_midpoint_rad_s[1]={-8.,5.,-11.};
    for(unsigned n=0;n<2;++n)in.position_endpoint_m[n]=tl::math::fixed3::Add(
      in.position_endpoint_m[n],tl::math::fixed3::Scale(in.velocity_midpoint_m_s[n],in.dt_s*double(step+1)));
    ASSERT_EQ(b::EvaluateForce(ref,material,accepted.proposed_history,in,next),b::Status::Success);
    const auto candidate=Native(ref,material,native.next,in);Compare(next,candidate);
    ASSERT_FALSE(::testing::Test::HasFailure());
    EXPECT_EQ(next.proposed_history.stamp().sample_index,step+1);
    accepted=next;native=candidate;
  }
}
TEST(Beam18ForceNative, SiAndWorkingUnitsCarryFourPointPlasticFilterFrameAndDamping) {
  Trajectory(Reference(),32);
  auto input=beam18_test::Input();b::Reference reference;
  ASSERT_EQ(b::InitializeReference(input,reference),b::Status::Success);Trajectory(reference,32);
}
TEST(Beam18ForceNative, ShortBeamAndSmallDtRetainDampingFloorAndRotationCoefficients) {
  for(double length:{.01,4.,20.}) {
    auto input=beam18_test::Input(length);b::Reference ref;
    ASSERT_EQ(b::InitializeReference(input,ref),b::Status::Success);
    const auto material=Material(ref);b::ForceTrial initial,next;
    ASSERT_EQ(b::InitializeForce(ref,material,{},initial),b::Status::Success);
    b::PrescribedInterval virgin;auto native=Native(ref,material,{},virgin,true);
    for(double dt:{1e-12,1e-10,1e-8}) {
      auto in=Motion(ref,initial.proposed_history);in.dt_s=dt;
      in.velocity_midpoint_m_s[1]={0,.2,-.1};in.angular_velocity_midpoint_rad_s[1]={1,2,3};
      ASSERT_EQ(b::EvaluateForce(ref,material,initial.proposed_history,in,next),b::Status::Success);
      Compare(next,Native(ref,material,native.next,in));ASSERT_FALSE(HasFailure());
    }
  }
}
#ifdef BEAM18_FORCE_ORIGINAL
TEST(Beam18ForceOriginal, All142SourceGeometriesUseOriginalMaterialAndCarriedNativeHistory) {
  ASSERT_EQ(std::size(beam18_test::original::Cells),142u);
  for(unsigned i=0;i<142;++i) {
    const auto input=beam18_test::original::Input(i);SCOPED_TRACE(input.source_element_id);
    b::Reference ref;ASSERT_EQ(b::InitializeReference(input,ref),b::Status::Success);
    Trajectory(ref,4);ASSERT_FALSE(HasFailure());
  }
}
#endif
} // namespace beam18_force_test
