#include "QephCoupledFixture.h"

namespace qeph_coupled_test {
bool NativeSequence::Initialize(const Rig& r) {
  for(unsigned n=0;n<r.n;++n) {
    for(unsigned a=0;a<3;++a) state.x[3*n+a]=r.x[3*n+a]; state.q[4*n]=1;
  }
  for(unsigned e=0;e<r.count;++e) {
    auto status=native::Initialize(qeph_startup_test::NativeInput(r.element[e].reference.input),reference_[e]);
    EXPECT_EQ(status,native::Status::kSuccess); if(status!=native::Status::kSuccess) return false;
    status=native::InitializeHistory(reference_[e],{0,0},cache[e].proposed_history);
    EXPECT_EQ(status,native::Status::kSuccess); if(status!=native::Status::kSuccess) return false;
    for(unsigned i=0;i<4;++i) { const auto n=r.element[e].nodes[i];
      mass[n]+=reference_[e].data().nodal_mass[i]; inertia[n]+=reference_[e].data().isotropic_inertia[i];
    }
  }
  for(unsigned n=0;n<r.n;++n) {
    EXPECT_NEAR(mass[n],r.mass[n],2e-12*mass[n]); EXPECT_NEAR(inertia[n],r.inertia[n],2e-12*inertia[n]);
  }
  return true;
}
bool NativeSequence::Propose(const Rig& r,const Loads& load,NativeProposal& output) const {
  NativeProposal p; p.state=state;
  // Native Q2 exposes positive internal force. Explicitly subtract the entire
  // accepted endpoint cache, then add prescribed world loads on shared nodes.
  for(unsigned e=0;e<r.count;++e) for(unsigned i=0;i<4;++i) {
    const auto n=r.element[e].nodes[i]; const auto f=cache[e].internal_force[i],c=cache[e].internal_couple[i];
    const double forces[]{f.x,f.y,f.z},couples[]{c.x,c.y,c.z};
    for(unsigned a=0;a<3;++a) { p.rhs.force[3*n+a]-=forces[a]; p.rhs.couple[3*n+a]-=couples[a]; }
  }
  const long double kick=epoch?r.h:.5L*r.h;
  for(unsigned n=0;n<r.n;++n) {
    double rotation[3]{};
    for(unsigned a=0;a<3;++a) { const auto i=3*n+a;
      p.rhs.force[i]+=load.force[i]; p.rhs.couple[i]+=load.couple[i];
      p.state.v[i]=static_cast<double>(state.v[i]+kick*p.rhs.force[i]/mass[n]);
      p.state.omega[i]=static_cast<double>(state.omega[i]+kick*p.rhs.couple[i]/inertia[n]);
      p.state.x[i]=static_cast<double>(static_cast<long double>(state.x[i])+r.h*static_cast<long double>(p.state.v[i]));
      rotation[a]=r.h*p.state.omega[i];
    }
    tl::math::Quaternion result;
    const auto* q0=state.q.data()+4*n;
    if(!tl::math::IncrementWorldRotation({q0[0],q0[1],q0[2],q0[3]},rotation,result)) return false;
    const double values[]{result.w,result.x,result.y,result.z};
    for(unsigned a=0;a<4;++a) p.state.q[4*n+a]=values[a];
  }
  for(unsigned e=0;e<r.count;++e) {
    const auto interval=Interval(r,e,p.state,time,epoch);
    const auto status=native::EvaluateForce(reference_[e],cache[e].proposed_history,
                                            qeph_kinematics_test::NativeInterval(interval),p.cache[e]);
    EXPECT_EQ(status,native::Status::kSuccess); if(status!=native::Status::kSuccess) return false;
  }
  output=p; return true;
}
} // namespace qeph_coupled_test
