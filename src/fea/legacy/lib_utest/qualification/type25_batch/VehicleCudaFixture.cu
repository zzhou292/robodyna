#include "VehicleCudaFixture.h"
#include <algorithm>
#include <cstring>

namespace type25_batch_test {
namespace {
__global__ void LoadFinal(fe::NodalAssemblyView view) {
  const auto n=view.accepted.node_count-1;
  view.forces.force_y[n]+=1e5;view.forces.couple_z[n]+=3;
}
}
bool VehicleRig::Initialize() {
  if(!input.Initialize())return false;
  for(std::size_t n=0;n<initial.n;++n) {
    const auto& x=input.mass.nodes()[n].position;
    initial.x[3*n]=x.x;initial.x[3*n+1]=x.y;initial.x[3*n+2]=x.z;
    initial.v[3*n]=0;initial.fixed[n]=initial.rotation_fixed[n]=0;
    initial.inverse[n]=1/input.mass.nodes()[n].coefficients.mass;
    initial.inertia[n]=1/input.mass.nodes()[n].coefficients.isotropic_inertia;
  }
  auto c=initial.config();c.fixed_dt=VehicleStep;
  const auto initialized=initial.Initialize(owner,c);EXPECT_EQ(initialized.status,fe::NodalStatus::Ok)<<initialized.message;
  if(initialized.status!=fe::NodalStatus::Ok)return false;
  const auto initialized_batch=batch.InitializeJoined(input.Config(owner.accepted()),input.model,input.mass,spring::CapacityProfile::Vehicle);
  EXPECT_EQ(initialized_batch.status,spring::BatchStatus::Success)<<initialized_batch.message;
  if(initialized_batch.status!=spring::BatchStatus::Success)return false;
  fe::NodalTrialToken token;fe::NodalAssemblyView view;
  if(owner.BeginTrial(&token,&view).status!=fe::NodalStatus::Ok)return false;
  const auto bound=batch.AssembleAccepted(owner,view);EXPECT_EQ(bound.status,spring::BatchStatus::Success)<<bound.message;
  Discard();return bound.status==spring::BatchStatus::Success;
}
bool VehicleRig::Prepare(fe::NodalTrialToken& token,fe::NodalPreparedView& prepared) {
  fe::NodalAssemblyView view;const auto begun=owner.BeginTrial(&token,&view);
  EXPECT_EQ(begun.status,fe::NodalStatus::Ok)<<begun.message;if(begun.status!=fe::NodalStatus::Ok)return false;
  LoadFinal<<<1,1,0,view.stream>>>(view);
  const auto assembled=batch.AssembleAccepted(owner,view);EXPECT_EQ(assembled.status,spring::BatchStatus::Success)<<assembled.message;
  if(assembled.status!=spring::BatchStatus::Success)return false;
  const auto sealed=owner.SealAssembly(token);EXPECT_EQ(sealed.status,fe::NodalStatus::Ok)<<sealed.message;
  if(sealed.status!=fe::NodalStatus::Ok)return false;
  const auto advanced=fe::AdvanceStaggeredHistory(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,VehicleStep,.1,8});
  EXPECT_EQ(advanced.status,fe::NodalStatus::Ok)<<advanced.message;
  return advanced.status==fe::NodalStatus::Ok&&owner.BorrowPrepared(token,&prepared).status==fe::NodalStatus::Ok;
}
bool VehicleRig::Read(std::vector<spring::Evaluation>& values,spring::BatchDiagnostics& diagnostics) {
  values.resize(input.model.connection_count());
  const auto r=batch.CopyAcceptedResults(owner.accepted(),values.data(),values.size(),&diagnostics);
  EXPECT_EQ(r.status,spring::BatchStatus::Success)<<r.message;return r.status==spring::BatchStatus::Success;
}
void ExactVehicle(const spring::Evaluation& a,const spring::Evaluation& b) {
  const auto x=type25_test::EvaluationValues(a),y=type25_test::EvaluationValues(b);
  for(std::size_t i=0;i<x.size();++i)EXPECT_EQ(std::memcmp(&x[i],&y[i],sizeof(double)),0)<<i;
  EXPECT_EQ(a.history.active,b.history.active);
}
void VehicleAgainstHost(VehicleRig& rig,const fe::NodalTrialToken& token,const fe::NodalPreparedView& prepared,
                        const std::vector<spring::Evaluation>& before,const std::vector<spring::Evaluation>& actual) {
  vehicle::Fields fields(rig.initial.n);fe::NodalPreparedView identity;
  ASSERT_EQ(rig.owner.CopyPrepared(token,fields.buffer(),&identity).status,fe::NodalStatus::Ok);
  ASSERT_TRUE(fe::trial_identity::SamePrepared(prepared,identity));
  for(std::size_t e=0;e<actual.size();++e) {
    const auto& c=rig.input.model.connections()[e];spring::EndpointKinematics nodes[2];
    for(unsigned a=0;a<2;++a) {
      const auto n=c.global_node[a];nodes[a]={{fields.x[3*n],fields.x[3*n+1],fields.x[3*n+2]},
        {fields.v[3*n],fields.v[3*n+1],fields.v[3*n+2]},{fields.w[3*n],fields.w[3*n+1],fields.w[3*n+2]}};
    }
    spring::Evaluation expected;
    ASSERT_EQ(spring::Evaluate(rig.input.model.source_units(),rig.input.model.properties()[c.property_index].property,
      rig.input.model.references()[e],before[e].history,nodes,VehicleStep,expected),spring::Status::Success)<<e;
    const auto av=type25_test::EvaluationValues(actual[e]),ev=type25_test::EvaluationValues(expected);
    for(std::size_t i=0;i<av.size();++i)EXPECT_NEAR(av[i],ev[i],2e-12*std::max(std::abs(ev[i]),1e-20))<<e<<":"<<i;
    EXPECT_EQ(actual[e].history.active,expected.history.active);
  }
}
} // namespace type25_batch_test
