#include "NativeAgreement.h"

namespace failure_force_test {
namespace {
double NativeArea(const native::Packet& n,const q::PrescribedInterval& interval) {
  native::nq::Kinematics k;
  if(native::nq::detail::UnpackKinematics(n.force.data(),n.planar,qeph_kinematics_test::NativeInterval(interval),k)!=native::nq::Status::kSuccess)
    throw std::runtime_error("native QEPH geometry packet");
  return k.area;
}
double NativeArea(const native::Packet& n,const t::PrescribedInterval& interval) {
  native::nt::Kinematics k;std::array<double,38> values;std::copy_n(n.force.begin(),38,values.begin());
  if(native::nt::detail::UnpackKinematics(values,t3_port_test::Native(interval),k)!=native::nt::Status::kSuccess)
    throw std::runtime_error("native T3 geometry packet");
  return k.area;
}
template<class F> void CoefficientAssociation() {
  using N=std::conditional_t<std::is_same_v<F,Q>,native::nq::Reference,native::nt::Reference>;
  for(unsigned mask:{0u,7u}) {
    Fixture<F> f(mask);f.failure.failure_strain=1;const auto reference=native::Reference<N>(f.reference.input);auto n=native::Seed(f.accepted);
    for(unsigned step=0;step<2;++step) {
      const auto interval=Interval(f.reference,step);
      native::Advance(reference,interval,f.material,f.failure.failure_strain,n);
      constexpr unsigned index=std::is_same_v<F,Q>?142:82;
      // Independently returned coefficient packet: THK0, SSP, DM. Use native
      // geometry as well; no production force/material helper computes this gate.
      const double ssp=n.force[index+1],dm=n.force[index+2];
      const double onep414=((1.+4./10.)+1./100.)+4./1000.;
      const double dtinv=interval.dt/std::max(interval.dt*interval.dt,1./1e20);
      const double expected=(onep414*dm)*ssp*std::sqrt(NativeArea(n,interval))*dtinv*f.reference.input.density;
      qeph_force_port_test::Independent(n.diagnostics[8],expected,2.e-11);
      const double nu=f.reference.input.poisson_ratio;
      const double substituted=expected/std::sqrt(1.-nu*nu);
      EXPECT_GT(std::abs(n.diagnostics[8]-substituted),.01*expected);
      constexpr unsigned active_index=std::is_same_v<F,Q>?37:25;
      EXPECT_EQ(n.history[active_index],mask==7?0:1);
    }
  }
}
}
TEST(ShellFailureForceNative,PreparedFamilySoundSpeedFeedsViscosityForActiveAndInactiveParents) {
  CoefficientAssociation<Q>();CoefficientAssociation<T>();
}
} // namespace failure_force_test
