// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeChecks.h"
#include "lib_utest/qualification/solid18_law44_startup/NativeOracle.h"
#include "lib_utest/qualification/law90_solid18_force/NativeSupport.h"
namespace extended_resident_test {
namespace rear=rear_force_test;
namespace foam=law90_force_test;
struct NativeChecks::State {
  const s::Model* model=nullptr;
  rear::NativeResult rear_accepted,rear_trial;
  fe::solid18::law44::ForceTrial rear_host,rear_next;
  foam::NativeForce foam_accepted{0},foam_trial{0};
  fe::solid18::total_strain::ForceTrial foam_host,foam_next;
  std::array<double,33> foam_parameters{};
};
NativeChecks::NativeChecks()=default;
NativeChecks::~NativeChecks()=default;
bool NativeChecks::Initialize(const s::Model& model,
    const tl::material::law90::PreparationInput& input,fe::solid18::Vec3 velocity) {
  if(model.solid18_law44().size()!=1||model.solid18_law90().size()!=1)return false;
  auto next=std::make_unique<State>();next->model=&model;
  const auto& a=model.solid18_law44()[0];const auto& b=model.solid18_law90()[0];
  const auto& ma=model.materials44()[a.material_index].value;
  const auto& mb=model.materials90()[b.material_index].value;
  next->rear_accepted=rear_startup_test::NativeInitialize(ma,a.reference.input(),velocity);
  if(next->rear_accepted.status)return false;
  if(fe::solid18::law44::InitializeForce(a.reference,ma,velocity,next->rear_host)!=fe::solid18::Status::Success)
    return false;
  next->foam_parameters=law90_point_test::NativePrepared(input,mb.curve());
  next->foam_accepted=foam::NativeForce(next->foam_parameters[1]);
  fe::solid18::PrescribedInterval initial;
  for(unsigned n=0;n<8;++n) {
    initial.position_endpoint_m[n]=b.reference.input().position_m[n];
    initial.velocity_midpoint_m_s[n]=velocity;
  }
  foam::AdvanceNative(next->foam_parameters.data(),mb.curve(),b.reference.input(),initial,true,next->foam_accepted);
  if(next->foam_accepted.status)return false;
  if(fe::solid18::total_strain::InitializeForce90(b.reference,mb,velocity,next->foam_host)!=fe::solid18::Status::Success)
    return false;
  state_=std::move(next);return true;
}
template<class Parent> fe::solid18::PrescribedInterval Packet(const Parent& parent,
    const std::vector<double>& x,const std::vector<double>& v,double base,double dt,std::uint64_t epoch) {
  fe::solid18::PrescribedInterval interval;
  interval.base_time_s=base;interval.dt_s=dt;interval.sample_index=epoch+1;
  for(unsigned n=0;n<8;++n) {
    const auto i=3*parent.domain_nodes[n];
    interval.position_endpoint_m[n]={x.at(i),x.at(i+1),x.at(i+2)};
    interval.velocity_midpoint_m_s[n]={v.at(i),v.at(i+1),v.at(i+2)};
  }
  return interval;
}
bool NativeChecks::Evaluate(const std::vector<double>& x,const std::vector<double>& v,
    double base,double dt,std::uint64_t epoch) {
  auto& s=*state_;const auto& model=*s.model;
  const auto& a=model.solid18_law44()[0];const auto& b=model.solid18_law90()[0];
  const auto& ma=model.materials44()[a.material_index].value;
  const auto& mb=model.materials90()[b.material_index].value;
  const auto pa=Packet(a,x,v,base,dt,epoch),pb=Packet(b,x,v,base,dt,epoch);
  s.rear_trial=rear::Native(ma,s.rear_accepted.next,pa);
  s.foam_trial=s.foam_accepted;
  foam::AdvanceNative(s.foam_parameters.data(),mb.curve(),b.reference.input(),pb,false,s.foam_trial);
  return s.rear_trial.status==0&&s.foam_trial.status==0&&
    fe::solid18::law44::EvaluateForce(a.reference,s.rear_host.proposed_history,pa,ma,s.rear_next)==fe::solid18::Status::Success&&
    fe::solid18::total_strain::EvaluateForce90(b.reference,s.foam_host.proposed_history,pb,mb,s.foam_next)==fe::solid18::Status::Success;
}
bool NativeChecks::Compare(const Results& result,bool candidate) {
  auto& s=*state_;
  auto a=candidate?s.rear_next:s.rear_host;
  auto b=candidate?s.foam_next:s.foam_host;
  const auto& na=candidate?s.rear_trial:s.rear_accepted;
  const auto& nb=candidate?s.foam_trial:s.foam_accepted;
  if(result.rear.size()!=1||result.foam.size()!=1)return false;
  const auto& ra=result.rear[0];const auto& rb=result.foam[0];
  EXPECT_EQ(ra.stamp.time_s,a.proposed_history.stamp().time_s);
  EXPECT_EQ(ra.stamp.sample_index,a.proposed_history.stamp().sample_index);
  EXPECT_EQ(rb.stamp.time_s,b.proposed_history.stamp().time_s);
  EXPECT_EQ(rb.stamp.sample_index,b.proposed_history.stamp().sample_index);
  // Existing complete native comparisons retain their original tolerances.
  // Replace every resident-exported value; only unretained force-call geometry
  // and point observations come from the independently checked host call.
  if(!rear::Agree(a,na)||!foam::ForceAgreement(foam::ForceValues(b),nb))return false;
  if(fe::solid18::law44::PreparePrescribedHistory(a.proposed_history.reference(),
      a.proposed_history.material(),ra.history,ra.stamp,a.proposed_history)!=fe::solid18::Status::Success)return false;
  if(fe::solid18::total_strain::PreparePrescribedHistory90(b.proposed_history.reference(),
      b.proposed_history.material(),rb.history,rb.stamp,b.proposed_history)!=fe::solid18::Status::Success)return false;
  a.diagnostics=ra.cache.diagnostics;b.diagnostics=rb.cache.diagnostics;
  for(unsigned n=0;n<8;++n) {
    a.rhs_force_n[n]=ra.cache.rhs_force_n[n];b.rhs_force_n[n]=rb.cache.rhs_force_n[n];
    b.point[n].material.history=rb.history.point[n];
    for(unsigned k=0;k<3;++k)EXPECT_EQ(rb.history.point[n].point.cursor[k],unsigned(nb.cursor[3*n+k]));
  }
  EXPECT_EQ(ra.cache.stiffness.translation_n_m,.25*ra.cache.diagnostics.raw_stiffness_n_m);
  EXPECT_EQ(rb.cache.stiffness.translation_n_m,.25*rb.cache.diagnostics.raw_stiffness_n_m);
  EXPECT_EQ(ra.cache.stiffness.rotation_nm,0);EXPECT_EQ(rb.cache.stiffness.rotation_nm,0);
  return rear::Agree(a,na)&&foam::ForceAgreement(foam::ForceValues(b),nb)&&!::testing::Test::HasFailure();
}
void NativeChecks::Commit() {
  auto& s=*state_;s.rear_accepted=s.rear_trial;s.foam_accepted=s.foam_trial;
  s.rear_host=s.rear_next;s.foam_host=s.foam_next;
}
} // namespace extended_resident_test
