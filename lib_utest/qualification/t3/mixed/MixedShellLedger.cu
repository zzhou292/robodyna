#include "MixedShellLedger.h"
#include "lib_utest/qualification/native/qeph/NativeQephBridge.h"
#include "lib_utest/qualification/native/t3/T3EngineContext.h"

extern "C" void qeph_q2_scatter(const double*,const double*,const int*,double*);
namespace mixed_shell_test {
namespace {
void Ledger(long double actual,long double expected,long double terms) {
  const long double budget=256*std::numeric_limits<double>::epsilon()*terms+1e-12L*EnergyScale;
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected)); ASSERT_TRUE(std::isfinite(budget));
  EXPECT_LE(std::abs(actual-expected),budget)<<std::setprecision(18)
      <<"actual="<<actual<<" expected="<<expected<<" budget="<<budget;
}
void Kinetic(const Rig& r,const Snapshot& state,const fe::ShellBatchKinetic& result) {
  // Independent world-area/atan2 mass oracle, reduced over the five global
  // nodes once. This is neither family's partial/global diagnostic sum.
  const auto mass=shell_binding_test::Truth(r.input);
  long double value[4]{};
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    const long double v=state.v[3*n+a],w=state.omega[3*n+a];
    value[0]+=.5L*mass[n].mass*v*v; value[1]+=.5L*mass[n].total*w*w;
    value[2]+=.5L*mass[n].physical*w*w; value[3]+=.5L*mass[n].added*w*w;
  }
  const double measured[]{result.translation,result.rotation,result.physical_isotropic,result.added_isotropic};
  for(unsigned i=0;i<4;++i) { SCOPED_TRACE(i); Ledger(measured[i],value[i],std::abs(value[i])); }
}
template<class Trial,class Diagnostic>
void SourceWork(const Trial& base,const Trial& next,const Diagnostic& d) {
  for(unsigned i=0;i<2;++i) {
    const long double a=base.proposed_history.data().internal_work[i],b=next.proposed_history.data().internal_work[i];
    const long double increment=next.diagnostics.internal_work_increment[i];
    const auto terms=std::abs(a)+std::abs(b)+std::abs(increment);
    Ledger(d.internal_work[i],b,terms); Ledger(d.internal_work_increment[i],increment,terms);
    Ledger(d.internal_work_increment[i],b-a,terms);
  }
  EXPECT_FALSE(d.accepted_force_assembled);
  EXPECT_EQ(d.internal_kick_work,0); EXPECT_EQ(d.internal_drift_work,0);
}
double Component(tl::math::Vec3 v,unsigned a) { return a==0?v.x:a==1?v.y:v.z; }
}
void CheckTargets(const Rig& r,unsigned interval,const Snapshot& endpoint) {
  ASSERT_LT(interval,4u);
  long double accumulated=0;
  for(unsigned i=0;i<=interval;++i) accumulated+=Targets[i];
  for(unsigned n=0;n<Nodes;++n) {
    const long double x=r.initial.x[3*n],y=r.initial.x[3*n+1];
    const long double v[]{.001L*(x+.25L*y),.0005L*(y-.5L*x),.00075L*(x-y)};
    const long double w[]{.002L*y,.003L*x,.001L*(x+y)};
    constexpr long double tolerance=temporal::ArithmeticTolerance;
    for(unsigned a=0;a<3;++a) {
      const long double expected_v=Targets[interval]*v[a],expected_w=Targets[interval]*w[a];
      const long double expected_x=r.initial.x[3*n+a]+H*accumulated*v[a];
      EXPECT_LE(std::abs(endpoint.v[3*n+a]-expected_v),tolerance*(1+std::abs(expected_v)));
      EXPECT_LE(std::abs(endpoint.omega[3*n+a]-expected_w),tolerance*(1+std::abs(expected_w)));
      EXPECT_LE(std::abs(endpoint.x[3*n+a]-expected_x),tolerance*(1+std::abs(expected_x)));
    }
    // Each node's prescribed world spin direction is fixed throughout the
    // sequence, so its rotations commute: this closed sin/cos result does not
    // call the owner's quaternion update or a second integration loop.
    const long double norm=std::sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);
    const long double half=.5L*H*accumulated*norm;
    const long double factor=norm?std::sin(half)/norm:0;
    const long double expected[]{std::cos(half),factor*w[0],factor*w[1],factor*w[2]};
    for(unsigned a=0;a<4;++a)
      EXPECT_LE(std::abs(endpoint.q[4*n+a]-expected[a]),tolerance*(1+std::abs(expected[a])));
  }
}
void CheckLedgers(const Rig& r,const Snapshot& base,const Staged& accepted,
                  const Prepared& p,const Staged& next) {
  Kinetic(r,base,next.diagnostics.base_kinetic); Kinetic(r,p.endpoint,next.diagnostics.kinetic);
  // Work of the prescribed RHS uses chosen native TOTAL m/J and the actual
  // velocity samples. Internal caches are deliberately not part of this kick.
  long double delta[2]{},work[2]{},terms[2]{};
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    const unsigned i=3*n+a;
    const long double mass[]{r.binding.nodes()[n].native.mass,r.binding.nodes()[n].native.isotropic_inertia};
    const long double old[]{base.v[i],base.omega[i]},now[]{p.endpoint.v[i],p.endpoint.omega[i]};
    const long double rhs[]{p.load.force[i],p.load.couple[i]};
    for(unsigned kind=0;kind<2;++kind) {
      const auto before=.5L*mass[kind]*old[kind]*old[kind],after=.5L*mass[kind]*now[kind]*now[kind];
      const auto supplied=p.view.kick_dt*rhs[kind]*(old[kind]+now[kind])*.5L;
      delta[kind]+=after-before; work[kind]+=supplied;
      terms[kind]+=std::abs(before)+std::abs(after)+std::abs(supplied);
    }
  }
  for(unsigned kind=0;kind<2;++kind) Ledger(delta[kind],work[kind],terms[kind]);
  SourceWork(accepted.qeph,next.qeph,next.diagnostics.qeph);
  SourceWork(accepted.t3,next.t3,next.diagnostics.t3);
  const long double a=accepted.qeph.proposed_history.data().hourglass_viscous_work;
  const long double b=next.qeph.proposed_history.data().hourglass_viscous_work;
  const long double increment=next.qeph.diagnostics.hourglass_viscous_work_increment;
  const auto absolute=std::abs(a)+std::abs(b)+std::abs(increment);
  Ledger(next.diagnostics.qeph.hourglass_viscous_work,b,absolute);
  Ledger(next.diagnostics.qeph.hourglass_viscous_work_increment,increment,absolute);
  Ledger(next.diagnostics.qeph.hourglass_viscous_work_increment,b-a,absolute);
  // T3 has no hourglass/EVIS channel; no zero surrogate is added to its type.
}
std::array<double,6*Nodes> Assembly(const fe::NodalAssemblyView& v) {
  std::array<double,6*Nodes> result{};
  const double* arrays[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                         v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for(unsigned c=0;c<6;++c)
    EXPECT_EQ(cudaMemcpyAsync(result.data()+c*Nodes,arrays[c],Nodes*sizeof(double),cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); return result;
}
void CheckNativeScatter(const Rig& r,const Staged& cache,const Loads& seed,const fe::NodalAssemblyView& view) {
  std::array<double,6*Nodes> expected{};
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    expected[a*Nodes+n]=seed.force[3*n+a]; expected[(a+3)*Nodes+n]=seed.couple[3*n+a];
  }
  // Complete retained CUPDTN3 on its existing local four-node buffer.
  double qvalues[24]{},qrhs[32]{};
  for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
    qvalues[3*i+a]=Component(cache.qeph.internal_force[i],a);
    qvalues[12+3*i+a]=Component(cache.qeph.internal_couple[i],a);
  }
  const double coefficients[]{cache.qeph.diagnostics.translational_stiffness,cache.qeph.diagnostics.rotational_stiffness,
      cache.qeph.kinematics.nodal_factors[0],cache.qeph.kinematics.nodal_factors[1]};
  const int qnodes[]{1,2,3,4};
  { const std::lock_guard<std::mutex> lock(qnative::detail::NativeContext());
    qeph_q2_scatter(qvalues,coefficients,qnodes,qrhs); }
  for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
    const auto n=r.binding.qeph_nodes()[i];
    expected[a*Nodes+n]+=qrhs[3*i+a]; expected[(a+3)*Nodes+n]+=qrhs[12+3*i+a];
  }
  // Complete retained C3UPDT3 remains within its FOUR-node bridge. Local
  // native nodes 1,2,3 are then mapped to the mixed five-node global union.
  const int tnodes[]{1,2,3}; const double k[2]{};
  double force[12]{},couple[12]{},stiffness[4]{},rotation[4]{},f[9]{},c[9]{};
  for(unsigned i=0;i<3;++i) for(unsigned a=0;a<3;++a) {
    f[3*i+a]=Component(cache.t3.internal_force[i],a); c[3*i+a]=Component(cache.t3.internal_couple[i],a);
  }
  { const std::lock_guard<std::mutex> lock(tnative::detail::NativeEngineContext());
    tnative::detail::t3_r3_scatter(tnodes,f,c,k,force,couple,stiffness,rotation); }
  for(unsigned i=0;i<3;++i) for(unsigned a=0;a<3;++a) {
    const auto n=r.binding.t3_nodes()[i];
    expected[a*Nodes+n]+=force[3*i+a]; expected[(a+3)*Nodes+n]+=couple[3*i+a];
  }
  EXPECT_EQ(Assembly(view),expected);
}
} // namespace mixed_shell_test
