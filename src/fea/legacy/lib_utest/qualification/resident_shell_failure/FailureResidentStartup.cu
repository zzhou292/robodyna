#include "FailureResidentFixture.h"

namespace resident_failure_test {
bool Initialize(Rig& rig,fe::ShellBatchPlasticityBinding& catalog,fe::ShellBatchFailureBinding& failure,
                bool enabled,bool mismatch,double failure_strain) {
  Source source;
  if(!source.Prepare(rig.binding,catalog))return false;
  rig.input=source.seed.geometry;
  rig.initial.n=Nodes;
  rig.initial.h=mixed::H;
  for(unsigned n=0;n<Nodes;++n) {
    const auto& node=rig.binding.nodes()[n];
    rig.initial.x[3*n]=node.position.x;
    rig.initial.x[3*n+1]=node.position.y;
    rig.initial.x[3*n+2]=node.position.z;
    rig.initial.inverse[n]=1/node.native.mass;
    rig.initial.inverse_inertia[n]=1/node.native.isotropic_inertia;
  }
  const auto state=rig.initial.Initialize(rig.owner);
  EXPECT_EQ(state.status,fe::NodalStatus::Ok);
  if(state.status!=fe::NodalStatus::Ok)return false;
  for (auto& row : source.failures) {
    if (row.policy == fe::ShellFailurePolicy::ConstantAllPoints) {
      row.constant.failure_strain = failure_strain;
    }
  }
  const auto binding=failure.Initialize(catalog,source.failures.data(),source.failures.size());
  EXPECT_EQ(binding.status,fe::ShellPlasticityBindingStatus::Success);
  if(binding.status!=fe::ShellPlasticityBindingStatus::Success)return false;
  qe::QephBatchConfig qc;
  qc.owner=rig.owner.accepted();
  qc.element_count=Parents;
  qc.configuration_id=mixed::Configuration;
  qc.qualification_id=mixed::Qualification;
  qc.usage=qe::BatchUsage::PrescribedFields;
  tr::T3BatchConfig tc;
  tc.owner=qc.owner;
  tc.element_count=Parents;
  tc.configuration_id=qc.configuration_id;
  tc.qualification_id=qc.qualification_id;
  tc.usage=tr::BatchUsage::PrescribedFields;
  const fe::ShellBatchFailureLimits limits;
  const auto qr=enabled?rig.qeph.InitializeJoined(qc,rig.binding,catalog,failure,limits):
    rig.qeph.InitializeJoined(qc,rig.binding,catalog);
  EXPECT_EQ(qr.status,qe::BatchStatus::Success)<<qr.message;
  fe::ShellBatchFailureBinding other;
  if(mismatch) {
    source.failures[2].constant.failure_strain*=2; // Only the OTHER family's declared policy changes.
    const auto changed=other.Initialize(catalog,source.failures.data(),source.failures.size());
    EXPECT_EQ(changed.status,fe::ShellPlasticityBindingStatus::Success);
    if(changed.status!=fe::ShellPlasticityBindingStatus::Success)return false;
  }
  const auto tt=enabled?rig.t3.InitializeJoined(tc,rig.binding,catalog,mismatch?other:failure,limits):
    rig.t3.InitializeJoined(tc,rig.binding,catalog);
  EXPECT_EQ(tt.status,tr::BatchStatus::Success)<<tt.message;
  source.failures[3].constant.failure_strain=-1; // Borrowed declarations expire here.
  return qr.status==qe::BatchStatus::Success&&tt.status==tr::BatchStatus::Success;
}
} // namespace resident_failure_test
