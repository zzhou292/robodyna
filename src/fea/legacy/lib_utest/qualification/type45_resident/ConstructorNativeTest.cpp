// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../type45_joint/NativeOracle.h"
#include "lib_src/elements/type45/resident/Virgin.h"
#include <limits>

extern "C" void type45_native_constructor(const int*,const double*,const double*,const int*,
    const double*,const double*,const double*,double*,double*,double*,int*);
namespace type45_test {
namespace {
struct ConstructorInput {
  std::array<double,15> xyz{};
  std::array<double,4> damping{};
  std::array<double,8> coefficient{};
  std::array<int,2> roles{};
  ConstructorInput(const Fixture& f,const NativeOracle& native) {
    for(unsigned i=0;i<5;++i) {
      const auto x=i<3?f.geometry.position_m[i]:f.context.main[i-3].position_m;
      for(unsigned a=0;a<3;++a) xyz[3*i+a]=detail::Get(x,a)/native.length;
    }
    for(unsigned i=0;i<2;++i) {
      roles[i]=static_cast<int>(f.context.main[i].role);
      damping[2*i]=f.damping[i].mass_kg/native.mass;
      damping[2*i+1]=f.damping[i].mean_principal_inertia_kg_m2/native.inertia;
      const auto& main=f.context.main[i];
      coefficient[4*i]=main.mass_kg/native.mass;
      coefficient[4*i+1]=main.inertia_kg_m2/native.inertia;
      coefficient[4*i+2]=main.translational_stiffness_n_m/native.mass;
      coefficient[4*i+3]=main.rotational_stiffness_nm/native.inertia;
    }
  }
};
}
TEST(Type45ConstructorNative,CompleteTt0CallerAllKindsBothUnitsAndOffsetRigidEndpoints) {
  for(auto kind:{Kind::Spherical,Kind::Revolute,Kind::Cylindrical})
    for(auto units:{WorkingUnits::SI,WorkingUnits::MillimetreTonneSecond}) {
      Fixture f(kind); f.property.working_units=units; f.property.automatic_stiffness_scale=.01;
      for(unsigned i=0;i<(kind==Kind::Spherical?2u:3u);++i)
        f.geometry.position_m[i]=f::Add(f.geometry.position_m[i],{1,-.5,.25});
      for(auto& main:f.context.main) main.position_m=f::Add(main.position_m,{1,-.5,.25});
      int status=-1; NativeOracle automatic(f,status); ASSERT_EQ(status,0);
      ConstructorInput packet(f,automatic);
      type45_native::State before; type45_native::Step result;
      const double spin[]{.2,-.3,.1,-.4,.1,.7};
      type45_native_constructor(&automatic.kind,automatic.property.data(),packet.xyz.data(),packet.roles.data(),
        packet.damping.data(),packet.coefficient.data(),spin,
        before.uvar.data(),before.history.data(),result.values.data(),&status);
      ASSERT_EQ(status,0);
      resident_detail::VirginCache actual;
      ASSERT_EQ(resident_detail::PrepareVirgin(f.property,f.geometry,actual),Status::Success);
      for(unsigned i=0;i<13;++i) EXPECT_EQ(before.history[i],0)<<i;
      for(unsigned i=0;i<20;++i) EXPECT_EQ(result.values[i],0)<<i; // Force, couple, STI/STIR and maxima.
      for(unsigned row=0;row<3;++row) for(unsigned col=0;col<3;++col)
        EXPECT_NEAR(actual.history.frame.v[3*row+col],before.uvar[21+3*col+row],2e-15);
      // The later TT0 assignment changes only automatic UVAR slots; native
      // cache/history above remains virgin. The positive-step oracle is intact.
      EXPECT_EQ(before.uvar[16],0); EXPECT_EQ(before.uvar[17],0);
      EXPECT_GT(automatic.state.uvar[16],0);
      for(unsigned i=0;i<39;++i) {
        if(i==16 || i==17 || (i>=18 && i<=20) || (i>=30 && i<=32)) continue;
        EXPECT_EQ(before.uvar[i],automatic.state.uvar[i])<<i;
      }
    }
}
TEST(Type45ConstructorNative,LateNonfiniteSpinAndOrdinaryZeroStepRejectWithoutOutputMutation) {
  Fixture f; int status=-1; NativeOracle native(f,status); ASSERT_EQ(status,0);
  ConstructorInput packet(f,native);
  type45_native::State state; state.uvar.fill(19); state.history.fill(23);
  type45_native::Step output; output.values.fill(29);
  const auto old=state; const auto old_output=output;
  double spin[6]{}; spin[5]=std::numeric_limits<double>::quiet_NaN();
  type45_native_constructor(&native.kind,native.property.data(),packet.xyz.data(),packet.roles.data(),
    packet.damping.data(),packet.coefficient.data(),spin,
    state.uvar.data(),state.history.data(),output.values.data(),&status);
  EXPECT_NE(status,0); EXPECT_EQ(state.uvar,old.uvar); EXPECT_EQ(state.history,old.history);
  EXPECT_EQ(output.values,old_output.values);
  spin[5]=0;
  const double time=0,dt=0; const int cycle=0;
  type45_native::type45_native_step(&native.kind,native.property.data(),packet.xyz.data(),spin,&time,&dt,&cycle,
      state.uvar.data(),state.history.data(),output.values.data(),&status);
  EXPECT_NE(status,0); EXPECT_EQ(state.history,old.history); EXPECT_EQ(output.values,old_output.values);
  type45_native_constructor(&native.kind,native.property.data(),packet.xyz.data(),packet.roles.data(),
    packet.damping.data(),packet.coefficient.data(),spin,
    state.uvar.data(),state.history.data(),output.values.data(),&status);
  EXPECT_EQ(status,0);
}
} // namespace type45_test
