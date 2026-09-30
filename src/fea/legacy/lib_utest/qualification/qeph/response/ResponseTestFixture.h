#pragma once
#include "ResponseSamples.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

namespace response_test {
namespace r=tl::qualification::qeph::response;
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& x) {
  std::array<unsigned char,sizeof(T)> b{}; std::memcpy(b.data(),&x,sizeof(T)); return b;
}
inline r::Model Model(unsigned cells=2) {
  r::Model m; std::string error; EXPECT_TRUE(r::BuildModel(cells,m,error))<<error; return m;
}
inline r::State Rest(const r::Model& m) {
  r::State s; s.x=m.initial_position; for(unsigned n=0;n<m.nodes;++n) s.q[4*n]=1; return s;
}
inline r::Results Histories(const r::Model& m,std::uint64_t epoch,double h) {
  r::Results out;
  for(unsigned e=0;e<m.cells;++e) {
    r::port::HistoryValues values; values.thickness=r::Thickness;
    EXPECT_EQ(r::port::PreparePrescribedHistory(m.reference[e],values,{epoch*h,epoch},out[e].proposed_history),r::port::Status::kSuccess);
    out[e].kinematics.area=m.reference[e].area;
  }
  return out;
}
inline unsigned Field(const std::vector<r::Field>& fields,const std::string& name) {
  for(unsigned i=0;i<fields.size();++i) if(fields[i].name==name) return i;
  ADD_FAILURE()<<"Missing field "<<name; return 0;
}
// Analytic report/comparison fixture only: not an owner trajectory or mechanics
// acceptance. The observer has separate actual-reference scalar-oracle tests.
inline r::Run Synthetic(unsigned refinement=1,unsigned cells=1) {
  r::Run out; out.config={cells,refinement}; out.model=Model(cells); out.fields=r::Dictionary(out.model);
  out.owner_id=71+refinement; out.completed=true; out.accepted_steps=out.attempted_steps=4096*refinement;
  out.samples.reserve(r::SampleCount); r::Sample initial; r::Limits limits; std::string error;
  EXPECT_TRUE(r::Observe(out.model,r::H0/refinement,0,Rest(out.model),Histories(out.model,0,r::H0/refinement),0,initial,limits,error));
  const double energy=r::ExperimentEnergy(out.model)*.001;
  for(unsigned j=0;j<r::SampleCount;++j) {
    const auto epoch=16*j*refinement; const double time=j*r::Horizon/256,h=r::H0/refinement;
    const double shape=std::sin(std::acos(-1.)*j/256);
    auto state=Rest(out.model); state.x[2]+=(.2+.008/refinement)*shape*r::Delta;
    auto cache=Histories(out.model,epoch,h); bool assigned[r::MaxNodes]{};
    for(unsigned e=0;e<cells;++e) for(unsigned i=0;i<4;++i) {
      const unsigned n=out.model.connectivity[e][i]; if(assigned[n]) continue; assigned[n]=true;
      // Cancel the known external pulse in this synthetic observation fixture.
      // It is not a material-consistent physical trajectory.
      const double factor=r::PulseFactor(time);
      if(cells==1) cache[e].internal_couple[i].y=(n/2?1.:-1.)*.5*r::BendingScale()*r::Theta*factor;
      else cache[e].internal_force[i].z=(n/2==1?-1.:.5)*r::BendingScale()*r::Delta/(r::Side*r::Side)*factor;
    }
    const double external_work=energy*std::min(1.,time/r::Pulse),residual=energy*.004/refinement*shape;
    auto values=cache[0].proposed_history.data(); values.internal_work[0]=external_work+residual;
    EXPECT_EQ(r::port::PreparePrescribedHistory(out.model.reference[0],values,{time,epoch},cache[0].proposed_history),r::port::Status::kSuccess);
    r::Sample s; r::Limits current;
    EXPECT_TRUE(r::Observe(out.model,h,epoch,state,cache,external_work,s,current,error))<<error;
    r::AccumulateLimits(out.observed,current);
    if(std::abs(s.residual)>out.maximum_abs_residual) { out.maximum_abs_residual=std::abs(s.residual); out.residual_time=s.time; }
    out.samples.push_back(s); const auto maximum=r::ResponseMaximum(out.fields,initial,s);
    if(maximum.value>out.maximum_response.value) out.maximum_response=maximum;
  }
  out.last_accepted=out.samples.back(); out.external_work_at_pulse=out.samples[64].external_work;
  return out;
}
} // namespace response_test
