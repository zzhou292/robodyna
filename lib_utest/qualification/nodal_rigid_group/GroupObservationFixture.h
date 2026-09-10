#pragma once
#include "GroupStepTestSupport.h"
#include "lib_src/constraints/NodalRigidKickObservation.h"
namespace rigid_observation_test {
namespace fe=tl::fea;
namespace rigid=fe::rigid;
using Vec3=tl::math::Vec3;
using Matrix=tl::math::Matrix3;
using Status=rigid::ObservationStatus;
using Motion=rigid::MemberMotion;
using Phase=rigid::ObservationPhase;
constexpr unsigned Count=4;
inline Phase InitialPhase() { return {rigid::ObservationPhaseKind::PhysicalInitialization,0,0,0}; }
inline Phase StoredPhase(double end,double frame) {
  return {rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,end,frame+.5*(end-frame),frame};
}
inline Matrix Multiply(Matrix a,Matrix b) {
  Matrix out{};
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) for(unsigned k=0;k<3;++k)
    out.v[3*i+j]+=a.v[3*i+k]*b.v[3*k+j];
  return out;
}
struct Fixture {
  enum class Kind { Dense,MeasurablePrimary,CorrectedInertia };
  std::array<fe::NodalRigidGroupMember,Count> source{};
  fe::NodalRigidGroupModel model;
  fe::NodalRigidGroupState state;
  std::array<Motion,Count> motion{};
  explicit Fixture(Kind kind=Kind::Dense) {
    const auto packet=rigid_step_test::Fixture();
    const double small_mass[]{2e-20,7e-20,3e-20,5e-20};
    for(unsigned i=0;i<Count;++i) {
      auto x=packet.member[i].position;
      if(kind==Kind::CorrectedInertia) x={double(i)-1.3,1e-6*(i%2),0};
      const double mass=kind==Kind::MeasurablePrimary?small_mass[i]:2;
      const double j=kind==Kind::MeasurablePrimary?2e-22:kind==Kind::CorrectedInertia?1e-8:.001;
      source[i]={100+i,i,x,mass,j,.4*j,.6*j};
    }
    fe::NodalRigidGroupInput group{300,400,source.data(),Count};
    const auto units=kind==Kind::MeasurablePrimary?fe::NodalRigidSourceUnits{1,1}:fe::NodalRigidSourceUnits{1000,.001};
    const auto result=model.Initialize({781,Count,&group,1,units}); EXPECT_TRUE(result)<<result.message;
    if(!result) return;
    state.center=model.groups()[0].center; state.velocity={.3,-.2,.1}; state.omega={.7,-1.2,.4};
    state.principal_axes=Multiply(rigid_step_test::DenseFrame().axes,model.groups()[0].principal.axes);
    SetRigidMotion();
  }
  rigid::GroupObservationMetric Metric() const { return {model.groups(),source.data(),Count}; }
  Vec3 CurrentArm(unsigned i) const {
    const auto ref=rigid::detail::Subtract(source[i].position,model.groups()[0].center);
    return rigid::detail::ToWorld(state.principal_axes,rigid::detail::ToLocal(model.groups()[0].principal.axes,ref));
  }
  void SetRigidMotion() {
    for(unsigned i=0;i<Count;++i) motion[i]={rigid::detail::Add(state.velocity,rigid::detail::Cross(state.omega,CurrentArm(i))),state.omega};
  }
  rigid::GroupKineticInput Input(Phase phase=StoredPhase(2,1)) const { return {Metric(),motion.data(),state,phase}; }
};
inline void Near(double actual,long double expected,long double tolerance=3e-12L) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  const auto scale=std::max({1e-300L,std::abs(expected),std::abs(static_cast<long double>(actual))});
  EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),tolerance*scale);
}
} // namespace rigid_observation_test
