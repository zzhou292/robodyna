// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type13/resident/Measurement.h"
#include "lib_src/elements/type25/Type25BatchMeasurement.h"
#include "FrozenType13Measure.h"
#include "FrozenType25Measure.h"
#include "FinalizeBodies.h"
#include <gtest/gtest.h>
#include <cfloat>
#include <cstring>
#include <vector>
namespace connector_operand_test {
namespace fe = tl::fea;
namespace a = fe::type13;
namespace b = fe::type25;
inline std::uint64_t Bits(double x) { std::uint64_t bits; std::memcpy(&bits, &x, sizeof(bits)); return bits; }
struct Type13 {
  using State = a::batch_detail::Storage; using Element = a::batch_detail::DeviceElement;
  using Evaluation = a::Evaluation; using Status = a::Status; using Diagnostics = a::BatchDiagnostics;
  using Measurement = a::batch_detail::Measurement; using Control = a::batch_detail::Control;
  static constexpr unsigned Channels = a::ChannelCount;
  static void Bind(State& s, std::size_t n, Evaluation* x, Evaluation* y) { s.model.element_count=n; s.slab[0]=x; s.slab[1]=y; }
  static double& Work(Evaluation& e, unsigned k) { return e.signed_work_J[k]; }
  static double& Dt(Evaluation& e) { return e.stability.critical_dt_s; }
  static bool& Active(Evaluation& e) { return e.native_history.active; }
  TL_SURFACE_HD static Measurement Prepare(const State& s, fe::NodalPreparedView v, std::size_t i) { return a::batch_detail::PrepareMeasurement(s,s.slab[0],s.slab[1],v,i); }
  TL_SURFACE_HD static void Reference(State& s, fe::NodalPreparedView v, Diagnostics d) { a::batch_detail::reference::Finalize(&s,0,1,v,d); }
  TL_SURFACE_HD static void Current(State& s, fe::NodalPreparedView v, Diagnostics d) { a::batch_detail::qualification_current::Finalize(&s,0,1,v,d); }
  static double& Kick(Diagnostics& d) { return d.internal_kick_work_J; }
  static double& Drift(Diagnostics& d) { return d.internal_drift_work_J; }
  static double Minimum(const Diagnostics& d) { return d.minimum_native_dt_s; }
};
struct Type25 {
  using State = b::batch_detail::Storage; using Element = b::batch_detail::DeviceElement;
  using Evaluation = b::Evaluation; using Status = b::Status; using Diagnostics = b::BatchDiagnostics;
  using Measurement = b::batch_detail::Measurement; using Control = b::batch_detail::Control;
  static constexpr unsigned Channels = 4;
  static void Bind(State& s, std::size_t n, Evaluation* x, Evaluation* y) { s.model.config.element_count=n; s.slab[0].element=x; s.slab[1].element=y; }
  static double& Work(Evaluation& e, unsigned k) { return e.history.internal_work_J[k]; }
  static double& Dt(Evaluation& e) { return e.critical_dt_s; }
  static bool& Active(Evaluation& e) { return e.history.active; }
  TL_SURFACE_HD static Measurement Prepare(const State& s, fe::NodalPreparedView v, std::size_t i) { return b::batch_detail::PrepareMeasurement(s,s.slab[0],s.slab[1],v,i); }
  TL_SURFACE_HD static void Reference(State& s, fe::NodalPreparedView v, Diagnostics d) { b::batch_detail::reference::Finalize(&s,&s.slab[0],&s.slab[1],v,d); }
  TL_SURFACE_HD static void Current(State& s, fe::NodalPreparedView v, Diagnostics d) { b::batch_detail::qualification_current::Finalize(&s,&s.slab[0],&s.slab[1],v,d); }
  static double& Kick(Diagnostics& d) { return d.internal_kick_work; }
  static double& Drift(Diagnostics& d) { return d.internal_drift_work; }
  static double Minimum(const Diagnostics& d) { return d.minimum_native_dt; }
};
template<class T> struct Fixture {
  typename T::State state;
  std::vector<typename T::Element> elements;
  std::vector<typename T::Evaluation> accepted, trial;
  std::vector<typename T::Status> status;
  std::vector<typename T::Measurement> operands;
  std::vector<double> x, x0, v, v0, omega, omega0;
  fe::NodalPreparedView view;
  typename T::Diagnostics seed;
  explicit Fixture(std::size_t count=17) : elements(count), accepted(count), trial(count),
      status(count,T::Status::Success), operands(count),x(12),x0(12),v(12),v0(12),omega(12),omega0(12) {
    T::Bind(state,count,accepted.data(),trial.data());
    state.model.elements=elements.data(); state.candidate_status=status.data(); state.measurement=operands.data();
    state.model.config.owner.fixed_dt=.125;
    view.kick_dt=.0625;
    view.base_kinematics.position_xyz=x0.data(); view.kinematics.position_xyz=x.data();
    view.base_kinematics.velocity_xyz=v0.data(); view.kinematics.velocity_xyz=v.data();
    view.base_kinematics.angular_velocity_xyz=omega0.data(); view.kinematics.angular_velocity_xyz=omega.data();
    for(std::size_t n=0;n<x.size();++n) {
      x0[n]=double(n)*.03125; x[n]=x0[n]+(n%2 ? -.25 : .125);
      v0[n]=n%2 ? -0.0 : 3; v[n]=n%3 ? -.5 : 1;
      omega0[n]=n%2 ? 7 : -2; omega[n]=n%3 ? .25 : -.125;
    }
    const double sensitive[]{1e16,1,-1e16,-0.0,DBL_MIN,-DBL_MIN,.25,-.25};
    for(std::size_t e=0;e<count;++e) {
      elements[e].nodes[0]=e%4; elements[e].nodes[1]=(e+1)%4;
      T::Dt(trial[e])=.01+double(e%3)*.001;
      T::Active(accepted[e])=true; T::Active(trial[e])=e%5!=0;
      for(unsigned k=0;k<T::Channels;++k) { T::Work(accepted[e],k)=double(k)*.25; T::Work(trial[e],k)=sensitive[(e+k)%8]; }
      for(unsigned n=0;n<2;++n) {
        const auto z=sensitive[(2*e+n)%8];
        accepted[e].endpoints[n].force_N={z,-z,z*.5};
        accepted[e].endpoints[n].couple_Nm={z*.25,z,-z};
        trial[e].endpoints[n].force_N={19,23,29}; trial[e].endpoints[n].couple_Nm={31,37,41};
      }
    }
    seed.source_instance_id=11; seed.owner_id=13; seed.configuration_id=17; seed.qualification_id=19;
    seed.epoch=5; seed.base_epoch=4; seed.attempt=7; seed.time=.001; seed.base_time=.0008;
    seed.velocity_time=.0009; seed.base_velocity_time=.0007; seed.kick_dt=.0002;
    seed.has_completed_interval=true; seed.accepted_force_assembled=true;
    T::Kick(seed)=.75; T::Drift(seed)=-0.0;
    for(unsigned k=0;k<T::Channels;++k) { seed.internal_work_J[k]=.5; seed.internal_work_increment_J[k]=-.25; }
  }
  void Stage() { for(std::size_t e=0;e<status.size();++e) operands[e]=T::Prepare(state,view,e); }
};
template<class T> void Same(const typename T::Control& x,const typename T::Control& y) {
  EXPECT_EQ(x.status,y.status); EXPECT_EQ(x.element_status,y.element_status);
  EXPECT_EQ(x.element,y.element); EXPECT_EQ(x.node,y.node);
  auto a=x.diagnostics,b=y.diagnostics;
#define EXACT_FIELD(f) EXPECT_EQ(a.f,b.f) << #f
  EXACT_FIELD(source_instance_id); EXACT_FIELD(owner_id); EXACT_FIELD(configuration_id); EXACT_FIELD(qualification_id);
  EXACT_FIELD(epoch); EXACT_FIELD(base_epoch); EXACT_FIELD(attempt); EXACT_FIELD(phase); EXACT_FIELD(valid);
  EXACT_FIELD(has_completed_interval); EXACT_FIELD(accepted_force_assembled);
  EXACT_FIELD(element_count); EXACT_FIELD(active_count); EXACT_FIELD(newly_failed_count);
#undef EXACT_FIELD
#define EXACT_DOUBLE(f) EXPECT_EQ(Bits(a.f),Bits(b.f)) << #f
  EXACT_DOUBLE(time); EXACT_DOUBLE(base_time); EXACT_DOUBLE(velocity_time); EXACT_DOUBLE(base_velocity_time); EXACT_DOUBLE(kick_dt);
#undef EXACT_DOUBLE
  for(unsigned k=0;k<T::Channels;++k) {
    EXPECT_EQ(Bits(a.internal_work_J[k]),Bits(b.internal_work_J[k]));
    EXPECT_EQ(Bits(a.internal_work_increment_J[k]),Bits(b.internal_work_increment_J[k]));
  }
  EXPECT_EQ(Bits(T::Kick(a)),Bits(T::Kick(b))); EXPECT_EQ(Bits(T::Drift(a)),Bits(T::Drift(b)));
  EXPECT_EQ(Bits(T::Minimum(a)),Bits(T::Minimum(b)));
}
template<class T> void Compare(Fixture<T>& f) {
  T::Reference(f.state,f.view,f.seed); const auto reference=f.state.control;
  f.Stage(); T::Current(f.state,f.view,f.seed); Same<T>(f.state.control,reference);
}
} // namespace connector_operand_test
