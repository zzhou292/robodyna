#pragma once
#include "T3BatchFixture.h"
#include "../../native/t3/T3EngineContext.h"

namespace t3_batch_test {
inline void CheckPrescribedTargets(const Rig& r,unsigned interval,const Snapshot& endpoint) {
  long double accumulated=0;
  for(unsigned k=0;k<=interval;++k) accumulated+=Targets[k];
  for(unsigned n=0;n<r.n;++n) {
    const long double x=r.x[3*n],y=r.x[3*n+1];
    const long double v[3]{.001L*(x+.25L*y),.0005L*(y-.5L*x),.00075L*(x-y)};
    const long double w[3]{.002L*y,.003L*x,.001L*(x+y)};
    for(unsigned a=0;a<3;++a) {
      const long double expected_v=Targets[interval]*v[a],expected_w=Targets[interval]*w[a];
      const long double expected_x=r.x[3*n+a]+H*accumulated*v[a];
      // Inherited actual-owner temporal fixture tolerance, fixed before run.
      constexpr long double tolerance=tl_test::nodal_temporal::ArithmeticTolerance;
      EXPECT_LE(std::abs(endpoint.v[3*n+a]-expected_v),tolerance*(1+std::abs(expected_v)));
      EXPECT_LE(std::abs(endpoint.omega[3*n+a]-expected_w),tolerance*(1+std::abs(expected_w)));
      EXPECT_LE(std::abs(endpoint.x[3*n+a]-expected_x),tolerance*(1+std::abs(expected_x)));
    }
  }
}
inline void Ledger(double value,long double expected,long double absolute_terms) {
  const long double budget=256*std::numeric_limits<double>::epsilon()*absolute_terms+1e-12L*EnergyScale;
  ASSERT_TRUE(std::isfinite(value));
  EXPECT_LE(std::abs(static_cast<long double>(value)-expected),budget)
      <<std::setprecision(18)<<value<<" expected "<<expected<<" budget "<<budget;
}
// Independent long-double reductions; native startup and force oracles remain
// separate from the production batch's reductions and source-work identities.
inline void CheckLedger(const Rig& r,const Results& base,const Results& next,
                        const Snapshot& endpoint,const t::BatchDiagnostics& d) {
  long double m[N]{},j[N]{},physical[N]{},added[N]{};
  for(unsigned e=0;e<r.count;++e) {
    const auto reference=native::test::Independent(oracle::Native(r.element[e].reference.input));
    for(unsigned i=0;i<3;++i) {
      const auto n=r.element[e].nodes[i];
      m[n]+=reference.mass*reference.weight[i]; j[n]+=reference.total*reference.weight[i];
      physical[n]+=reference.physical*reference.weight[i]; added[n]+=reference.added*reference.weight[i];
    }
  }
  long double kinetic=0,rotational=0,kp=0,ka=0;
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    const long double v=endpoint.v[3*n+a],w=endpoint.omega[3*n+a];
    kinetic+=.5L*m[n]*v*v; rotational+=.5L*j[n]*w*w;
    kp+=.5L*physical[n]*w*w; ka+=.5L*added[n]*w*w;
  }
  Ledger(d.kinetic_translation,kinetic,std::abs(kinetic)); Ledger(d.kinetic_rotation,rotational,std::abs(rotational));
  Ledger(d.kinetic_physical_isotropic,kp,std::abs(kp)); Ledger(d.kinetic_added_isotropic,ka,std::abs(ka));
  for(unsigned c=0;c<2;++c) {
    long double total=0,increment=0,difference=0,terms=0;
    for(unsigned e=0;e<r.count;++e) {
      const long double a=base[e].proposed_history.data().internal_work[c],b=next[e].proposed_history.data().internal_work[c];
      total+=b; increment+=next[e].diagnostics.internal_work_increment[c]; difference+=b-a;
      terms+=std::abs(a)+std::abs(b)+std::abs(next[e].diagnostics.internal_work_increment[c]);
    }
    Ledger(d.internal_work[c],total,terms); Ledger(d.internal_work_increment[c],increment,terms);
    Ledger(d.internal_work_increment[c],difference,terms);
  }
  EXPECT_EQ(d.internal_kick_work,0); EXPECT_EQ(d.internal_drift_work,0); // PrescribedFields only.
}
inline std::array<double,6*N> Assembly(const fe::NodalAssemblyView& v) {
  std::array<double,6*N> out{};
  const double* source[6]{v.forces.force_x,v.forces.force_y,v.forces.force_z,v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for(unsigned c=0;c<6;++c)
    EXPECT_EQ(cudaMemcpyAsync(out.data()+c*N,source[c],v.forces.node_count*sizeof(double),cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); return out;
}
inline void CheckNativeScatter(const Rig& r,const Results& cache,const fe::NodalAssemblyView& view) {
  std::array<double,12> force{},couple{}; std::array<double,4> stiffness{},rotation{};
  // Complete retained C3UPDT3 native leaf, seeded independent nonzero RHS.
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    force[3*n+a]=.125*(1+3*n+a); couple[3*n+a]=-.0625*(1+3*n+a);
  }
  for(unsigned e=0;e<r.count;++e) {
    std::array<int,3> nodes{}; std::array<double,9> f{},c{}; const double k[2]{};
    for(unsigned i=0;i<3;++i) { nodes[i]=int(r.element[e].nodes[i])+1;
      for(unsigned a=0;a<3;++a) {
        f[3*i+a]=oracle::Component(cache[e].internal_force[i],a);
        c[3*i+a]=oracle::Component(cache[e].internal_couple[i],a);
      }
    }
    const std::lock_guard<std::mutex> lock(native::detail::NativeEngineContext());
    native::detail::t3_r3_scatter(nodes.data(),f.data(),c.data(),k,force.data(),couple.data(),stiffness.data(),rotation.data());
  }
  const auto actual=Assembly(view);
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    EXPECT_EQ(actual[a*N+n],force[3*n+a]); EXPECT_EQ(actual[(a+3)*N+n],couple[3*n+a]);
  }
}
} // namespace t3_batch_test
