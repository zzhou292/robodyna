// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeLengthRoundoff.h"
#include <boost/multiprecision/cpp_bin_float.hpp>

namespace type45_test {
namespace {
using Wide=boost::multiprecision::number<boost::multiprecision::cpp_bin_float<100>>;
namespace d=tl::fea::type45::detail;
Wide Projection(const double* frame,bool native,const Vec3* positions,unsigned axis,double length) {
  Wide sum=0;
  for(unsigned j=0;j<3;++j) {
    const double a=d::Get(positions[0],j)/length,b=d::Get(positions[1],j)/length;
    sum+=Wide(frame[native ? 3*axis+j : 3*j+axis])*(Wide(b)-Wide(a));
  }
  return sum;
}
Wide Force(const Wide& displacement,const Wide& old,const Wide& k,
           const Wide& m1,const Wide& m2,const Wide& ratio,const Wide& viscosity,double dt) {
  // Direct scalar constitutive definition, independently at 100 decimal digits.
  return k*displacement+(ratio*sqrt(k*m1*m2/(m1+m2))+viscosity)*(displacement-old)/Wide(dt);
}
} // namespace
TEST(Type45Native, PreciseRepresentedPacketsExplainLengthSensitivityAndRejectForceCorruption) {
  for(unsigned shifted=0;shifted<2;++shifted) {
    Fixture fixture(Kind::Spherical);
    fixture.property.working_units=WorkingUnits::MillimetreTonneSecond;
    if(shifted) {
      // Large original-coordinate scale; preserve the identical relative geometry.
      const Vec3 offset{1.003454234,.4365672,-.2746315};
      for(unsigned i=0;i<2;++i) {
        fixture.geometry.position_m[i]=f::Add(fixture.geometry.position_m[i],offset);
        fixture.context.main[i].position_m=f::Add(fixture.context.main[i].position_m,offset);
      }
    }
    int status=-1;
    NativeOracle native(fixture,status);
    ASSERT_EQ(status,0);
    const auto reference=fixture.Prepare();
    CompareReference(fixture,reference,native);
    History accepted;
    ASSERT_EQ(History::Initialize(reference,accepted),Status::Success);
    for(unsigned step=0;step<8;++step) {
      SCOPED_TRACE(step);
      auto input=fixture.Step(accepted);
      input.position_m[1].x+=2e-5*::sin(.2*(step+1));
      input.position_m[1].y-=3e-5*::cos(.17*(step+1));
      input.angular_velocity_rad_s[0]={.13,-.07,.03};
      input.angular_velocity_rad_s[1]={.23,.19,-.11};
      Evaluation actual;
      ASSERT_EQ(Evaluate(reference,accepted,input,actual),Status::Success);
      type45_native::Step expected;
      ASSERT_TRUE(native.Step(input,expected));
      const auto bound=CheckLengthRoundoff(actual,native,expected);
      for(unsigned i=0;i<3;++i) {
        SCOPED_TRACE(i);
        const Wide dp=Projection(actual.history.values().frame.v,false,input.position_m,i,1)-
          Projection(reference.frame().v,false,fixture.geometry.position_m,i,1);
        const Wide dn=Projection(native.state.uvar.data()+21,true,input.position_m,i,native.length)-
          Projection(native.initial_state.uvar.data()+21,true,fixture.geometry.position_m,i,native.length);
        const Wide fp=Force(dp,Wide(d::Get(accepted.values().local_displacement_m,i)),
          Wide(d::Get(reference.stiffness().translation,i)),Wide(fixture.damping[0].mass_kg),
          Wide(fixture.damping[1].mass_kg),Wide(fixture.property.critical_damping_ratio),Wide(0),input.dt_s);
        const Wide fn=Force(dn,Wide(native.previous_state.history[i]),Wide(native.state.uvar[18+i]),
          Wide(native.state.uvar[33]),Wide(native.state.uvar[34]),Wide(native.property[1]),Wide(0),input.dt_s)*Wide(native.force);
        EXPECT_LE(abs(Wide(d::Get(actual.history.values().local_force_n,i))-fp),
                  Wide(d::Get(bound.projection_force_error[0],i)));
        EXPECT_LE(abs(Wide(native.state.history[6+i])*Wide(native.force)-fn),
                  Wide(d::Get(bound.projection_force_error[1],i)));
        const double value=d::Get(actual.endpoint[0].force_n,i),target=expected.values[i]*native.force;
        const double moment=d::Get(actual.endpoint[0].couple_nm,i),target_m=expected.values[3+i]*native.inertia;
        EXPECT_TRUE(WithinLengthBound(value,target,d::Get(bound.world_force,i)));
        EXPECT_FALSE(WithinLengthBound(value+1e-4,target,d::Get(bound.world_force,i)));
        EXPECT_TRUE(WithinLengthBound(moment,target_m,d::Get(bound.endpoint_couple[0],i)));
        EXPECT_FALSE(WithinLengthBound(moment+1e-6,target_m,d::Get(bound.endpoint_couple[0],i)));
      }
      CompareStep(actual,native,expected);
      accepted=actual.history;
    }
  }
}
} // namespace type45_test
