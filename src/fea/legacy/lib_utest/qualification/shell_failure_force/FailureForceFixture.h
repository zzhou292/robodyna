#pragma once
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "../shell_layered_failure/FailureSectionFixture.h"
#include "lib_src/elements/qeph/QephLayeredJ2Failure.h"
#include "lib_src/elements/t3/T3LayeredJ2Failure.h"
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#if defined(__CUDACC__)
#define FAILURE_TEST_HD __host__ __device__
#else
#define FAILURE_TEST_HD
#endif
namespace failure_force_test {
namespace q=tl::fea::qeph;namespace t=tl::fea::t3;namespace sec=tl::fea::sections;
using tl::math::Vec3;
constexpr double Dt=0x1p-20;
FAILURE_TEST_HD inline Vec3 Position(Vec3 x,double time) {
  return {x.x+time*(2000*x.x+300*x.y),x.y-time*500*x.y,
      x.z+time*(20000*x.x*x.x-12000*x.y*x.y+3000*x.x*x.y)};
}
FAILURE_TEST_HD inline Vec3 Velocity(Vec3 x) {return {2000*x.x+300*x.y,-500*x.y,20000*x.x*x.x-12000*x.y*x.y+3000*x.x*x.y};}
FAILURE_TEST_HD inline Vec3 Spin(Vec3 x) {return {24000*x.y-3000*x.x,40000*x.x+3000*x.y,70};}
FAILURE_TEST_HD inline q::PrescribedInterval Interval(const q::ReferenceData& r,unsigned step) {
  q::PrescribedInterval in;in.base_time=step*Dt;in.dt=Dt;in.sample_index=step+1;
  for(unsigned n=0;n<4;++n) {const auto x=r.input.position[n];in.position_endpoint[n]=Position(x,(step+1)*Dt);
    in.velocity_midpoint[n]=Velocity(x);in.omega_midpoint[n]=Spin(x);}
  return in;
}
FAILURE_TEST_HD inline t::PrescribedInterval Interval(const t::ReferenceData& r,unsigned step) {
  t::PrescribedInterval in;in.base_time=step*Dt;in.dt=Dt;in.sample_index=step+1;
  for(unsigned n=0;n<3;++n) {const auto x=r.input.position[n];in.position[n]=Position(x,(step+1)*Dt);
    in.velocity[n]=Velocity(x);in.angular_velocity[n]=Spin(x);}
  return in;
}
struct Q {
  using Status=q::Status;
  using Reference=q::ReferenceData;using History=q::LayeredJ2FailureHistory;using Trial=q::LayeredJ2FailureForceTrial;
  using LegacyHistory=q::LayeredJ2History;using LegacyTrial=q::LayeredJ2ForceTrial;
  static Reference ReferenceValue() {
    q::ReferenceInput in;in.thickness=.002;
    in.position[0]={0,0,0};in.position[1]={.02,0,0};in.position[2]={.02,.02,.0002};in.position[3]={0,.02,0};Reference r;
    if(q::InitializeReference(in,r)!=q::Status::kSuccess)throw std::runtime_error("Q reference");return r;}
};
struct T {
  using Status=t::Status;
  using Reference=t::ReferenceData;using History=t::LayeredJ2FailureHistory;using Trial=t::LayeredJ2FailureForceTrial;
  using LegacyHistory=t::LayeredJ2History;using LegacyTrial=t::LayeredJ2ForceTrial;
  static Reference ReferenceValue() {
    t::ReferenceInput in;in.thickness=.002;in.position[0]={0,0,0};in.position[1]={.02,0,0};in.position[2]={0,.02,0};Reference r;
    if(t::InitializeReference(in,r)!=t::Status::kSuccess)throw std::runtime_error("T reference");return r;
  }
};
template<class F> struct Fixture {
  sec::PointParameters material=layered_failure_test::Parameters();
  sec::ConstantFailureParameters failure{.003};
  typename F::Reference reference=F::ReferenceValue();typename F::History accepted;
  explicit Fixture(unsigned mask=0) {
    if(InitializeLayeredJ2FailureHistory(reference,material,failure,{},accepted)!=F::Status::kSuccess)
      throw std::runtime_error("failure history");
    accepted.section=layered_failure_test::Seed(mask);
    for(auto& h:accepted.section.failure) h.failure_time_s=0;
    auto shell=accepted.shell.data();shell.active=accepted.section.element_active?1:0;
    if(PrepareFailurePrescribedHistory(reference,shell,{},accepted.shell)!=F::Status::kSuccess)
      throw std::runtime_error("failure mask");
  }
  auto Evaluate(unsigned step,typename F::Trial& out) const {
    return EvaluateLayeredJ2FailureForce(reference,material,failure,accepted,Interval(reference,step),out);
  }
  void Accept(const typename F::Trial& trial) {accepted={trial.force.proposed_history,trial.section.history};}
};
// Scalar inventories avoid comparing unspecified aggregate padding.
inline void Append(std::vector<double>& out,double x) {out.push_back(x);}
inline void Append(std::vector<double>& out,Vec3 v) {Append(out,v.x);Append(out,v.y);Append(out,v.z);}
template<class V,std::size_t N> void Append(std::vector<double>& out,const V (&v)[N]) {for(const auto& x:v)Append(out,x);}
template<class H> std::vector<double> CommonHistory(const H& h) {
  std::vector<double> v;Append(v,h.stress);Append(v,h.material_stress);Append(v,h.bending_stress);
  Append(v,h.strain_curvature);Append(v,h.thickness);Append(v,h.internal_work);Append(v,h.active);return v;
}
inline std::vector<double> Values(const q::HistoryValues& h) {auto v=CommonHistory(h);Append(v,h.stabilization);Append(v,h.hourglass_viscous_work);return v;}
inline std::vector<double> Values(const t::HistoryValues& h) {auto v=CommonHistory(h);Append(v,h.equivalent_strain_rate);return v;}
inline void Exact(const std::vector<double>& a,const std::vector<double>& b) {
  ASSERT_EQ(a.size(),b.size());for(std::size_t i=0;i<a.size();++i)EXPECT_EQ(std::memcmp(&a[i],&b[i],sizeof(double)),0)<<i;
}
} // namespace failure_force_test
