#include "WallIncomingFixture.h"
#include "../wall_coupled/ScreenedWallFixture.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace qeph_wall_incoming_test {
namespace {
using Wide=long double;
constexpr Wide Roundoff=256*std::numeric_limits<double>::epsilon();
unsigned Mask(const sc::NodalWallDeviceResults& contact,const Snapshot& state,unsigned nodes) {
  unsigned mask=0; EXPECT_EQ(contact.diagnostics.node_count,nodes);
  for(unsigned i=0;i<nodes;++i) {
    const auto& p=contact.nodes[i]; EXPECT_EQ(p.node,i);
    EXPECT_EQ(p.touching_or_penetrating,state.x[3*i]>=0);
    if(p.touching_or_penetrating) mask|=1u<<i;
  }
  EXPECT_TRUE(mask==0||mask==((1u<<nodes)-1)); return mask;
}
void ZeroResponse(const IncomingRig& w,const Trial& t,const Staged& s,IncomingStage& evidence) {
  CheckScreenedRigid(w.coupled,t,s,w.scales,evidence.maximum_regular_rate_ratio,evidence.maximum_hourglass_rate_ratio);
}
void Controls(const IncomingRig& w,const Snapshot& base,const Trial& t,const Staged& s,
              const PortResults& cache,IncomingStage& out) {
  const auto& r=w.coupled.shell; const auto& p=t.nodal;
  Loads next; std::array<Wide,N> source_uncertainty{};
  for(unsigned e=0;e<r.count;++e) {
    const auto interval=Interval(r,e,p.state,p.view.base_time,base.stamp.epoch);
    const auto& input=r.element[e].reference.input;
    const double scale=input.young_modulus*input.thickness*qeph_kinematics_test::LengthScale(interval);
    for(unsigned i=0;i<4;++i) {
      const auto n=r.element[e].nodes[i]; const double f=s.elements[e].internal_force[i].x;
      next.force[3*n]-=f;
      source_uncertainty[n]+=2e-12L*(scale+std::abs(f))+2e-12L*(scale+std::abs(cache[e].internal_force[i].x));
    }
  }
  for(unsigned n=0;n<r.n;++n) {
    const auto& applied=t.contact_base_result.nodes[n]; const auto& candidate=s.contact_result.nodes[n];
    next.force[3*n]+=candidate.force_world.x;
    const Wide kick=p.view.kick_dt,inverse=r.inverse[n],v0=base.v[3*n],actual=p.state.v[3*n];
    const Wide omitted=v0+kick*inverse*(static_cast<Wide>(p.assembled.force[3*n])-applied.force_world.x);
    const Wide premature=v0+kick*inverse*next.force[3*n];
    const Wide roundoff=Roundoff*(std::abs(v0)+std::abs(actual)+std::abs(omitted)+std::abs(premature));
    const Wide base_budget=VelocityBudget()+roundoff+kick*inverse*(applied.force.error+source_uncertainty[n]);
    const Wide next_budget=VelocityBudget()+roundoff+kick*inverse*(applied.force.error+candidate.force.error+source_uncertainty[n]);
    const Wide omitted_signal=std::abs(actual-omitted),early_signal=std::abs(actual-premature);
    ASSERT_TRUE(std::isfinite(base_budget)&&base_budget>0);
    ASSERT_TRUE(std::isfinite(next_budget)&&next_budget>0);
    if(omitted_signal/base_budget>out.omitted_contact) {
      out.omitted_contact=static_cast<double>(omitted_signal/base_budget);
      out.omitted_signal=static_cast<double>(omitted_signal); out.omitted_budget=static_cast<double>(base_budget);
    }
    if(early_signal/next_budget>out.premature_endpoint) {
      out.premature_endpoint=static_cast<double>(early_signal/next_budget);
      out.premature_signal=static_cast<double>(early_signal); out.premature_budget=static_cast<double>(next_budget);
    }
  }
}
}
void CheckInitial(const IncomingRig& w,const Snapshot& state,const PortResults& cache,const q::BatchDiagnostics& d) {
  CheckScreenedInitial(w.coupled,w.scales,state,cache,d);
}
bool CheckIncoming(const IncomingRig& w,const Snapshot& base,const Trial& t,const Staged& s,const PortResults& cache,
                   const NativeProposal& native,const IncomingEvidence& prior,IncomingStage& output) {
  const auto& r=w.coupled.shell; IncomingStage next; next.coupled=s;
  next.source=prior.source; next.contact=prior.contact;
  next.omitted_contact=prior.omitted_contact; next.premature_endpoint=prior.premature_endpoint;
  next.omitted_signal=prior.omitted_signal; next.omitted_budget=prior.omitted_budget;
  next.premature_signal=prior.premature_signal; next.premature_budget=prior.premature_budget;
  next.maximum_regular_rate_ratio=prior.maximum_regular_rate_ratio; next.maximum_hourglass_rate_ratio=prior.maximum_hourglass_rate_ratio;
  Identity(w.coupled,t,s); Agreement(w.coupled,t,s,native,base.stamp.time,base.stamp.epoch);
  const auto host=w.coupled.Host(base,base.stamp.epoch,t.nodal.view.attempt);
  ContactAgreement(t.contact_base_result,host);
  CheckLedgers(r,base,t.nodal,ContactLoads(host),cache,s.shell,w.scales,next.source);
  CheckSourceWork(r,cache,s.elements,s.shell,w.scales.energy,next.source);
  ContactLedgers(w.coupled,base,t,s,w.scales,next.contact);
  auto& event=next.event; event.base_epoch=base.stamp.epoch; event.base_time=base.stamp.time;
  event.endpoint_time=t.nodal.view.proposed_time; event.base_mask=Mask(t.contact_base_result,base,r.n);
  event.endpoint_mask=Mask(s.contact_result,t.nodal.state,r.n); const unsigned full=(1u<<r.n)-1;
  event.minimum_base_gap=event.maximum_base_gap=base.x[0];
  event.minimum_endpoint_gap=event.maximum_endpoint_gap=t.nodal.state.x[0];
  event.minimum_velocity=event.maximum_velocity=t.nodal.state.v[0];
  for(unsigned n=0;n<r.n;++n) {
    event.minimum_base_gap=std::min(event.minimum_base_gap,base.x[3*n]); event.maximum_base_gap=std::max(event.maximum_base_gap,base.x[3*n]);
    event.minimum_endpoint_gap=std::min(event.minimum_endpoint_gap,t.nodal.state.x[3*n]); event.maximum_endpoint_gap=std::max(event.maximum_endpoint_gap,t.nodal.state.x[3*n]);
    event.minimum_velocity=std::min(event.minimum_velocity,t.nodal.state.v[3*n]); event.maximum_velocity=std::max(event.maximum_velocity,t.nodal.state.v[3*n]);
  }
  event.crossing=event.base_mask==0&&event.endpoint_mask==full;
  if(event.base_mask==0) EXPECT_EQ(t.contact_base.potential.upper,0);
  if(event.crossing) {
    EXPECT_EQ(prior.crossing_base,UINT64_MAX); EXPECT_GT(event.minimum_endpoint_gap,0);
    EXPECT_GT(s.contact.potential.lower,0); EXPECT_EQ(t.contact_base.resultant.upper,0);
    EXPECT_GE(event.base_epoch+1,w.schedule.windows.front().entry_base_epoch);
    EXPECT_LE(event.base_epoch+1,w.schedule.windows.back().entry_base_epoch);
    for(unsigned n=0;n<r.n;++n) EXPECT_NEAR(t.nodal.state.v[3*n],wr::ImpactSpeed,VelocityBudget());
  }
  if(prior.crossing_base!=UINT64_MAX) EXPECT_EQ(event.base_mask,full);
  ZeroResponse(w,t,s,next); Controls(w,base,t,s,cache,next);
  if(event.crossing) EXPECT_GT(next.premature_endpoint,32);
  if(prior.crossing_base!=UINT64_MAX&&event.base_epoch==prior.crossing_base+1) {
    EXPECT_GT(t.contact_base.resultant.lower,0); EXPECT_LT(event.maximum_velocity,wr::ImpactSpeed);
    EXPECT_GT(next.omitted_contact,32);
  }
  EXPECT_LT(prior.count,MaximumPrefixSteps); EXPECT_EQ(prior.count,event.base_epoch);
  if(::testing::Test::HasFailure()) return false;
  next.checked=true; output=next; return true;
}
bool PublishIncoming(IncomingRig& w,const Trial& t,const IncomingStage& stage,
                     sc::NodalWallDeviceResults& published,IncomingEvidence& evidence) {
  if(!stage.checked||evidence.count>=MaximumPrefixSteps||evidence.count!=stage.event.base_epoch) return false;
  if(!Publish(w.coupled,t,stage.coupled,published)) return false;
  evidence.accepted[evidence.count++]=stage.event;
  evidence.source=stage.source; evidence.contact=stage.contact;
  evidence.omitted_contact=stage.omitted_contact; evidence.premature_endpoint=stage.premature_endpoint;
  evidence.omitted_signal=stage.omitted_signal; evidence.omitted_budget=stage.omitted_budget;
  evidence.premature_signal=stage.premature_signal; evidence.premature_budget=stage.premature_budget;
  evidence.maximum_regular_rate_ratio=stage.maximum_regular_rate_ratio; evidence.maximum_hourglass_rate_ratio=stage.maximum_hourglass_rate_ratio;
  if(stage.event.crossing) evidence.crossing_base=stage.event.base_epoch;
  if(stage.event.base_mask&&evidence.first_applied_base==UINT64_MAX) evidence.first_applied_base=stage.event.base_epoch;
  return true;
}
void CheckFinished(const IncomingRig& w,const IncomingEvidence& e) {
  EXPECT_EQ(e.count,w.intervals); EXPECT_NE(e.crossing_base,UINT64_MAX);
  EXPECT_EQ(e.first_applied_base,e.crossing_base+1); EXPECT_GT(e.omitted_contact,32); EXPECT_GT(e.premature_endpoint,32);
  EXPECT_LE(e.maximum_regular_rate_ratio,1); EXPECT_LE(e.maximum_hourglass_rate_ratio,1);
  if(e.count<MaximumPrefixSteps) EXPECT_EQ(e.accepted[e.count].base_epoch,0u);
}
} // namespace qeph_wall_incoming_test
