#include "ResidentCollectionFixture.h"

namespace resident_plasticity_test {
namespace {
bool InitializeCollectionImpl(Rig& r,fe::ShellBatchPlasticityBinding& oracle,
    bool mismatch_other_family,bool legacy_triangle,unsigned analytic) {
  plasticity_binding_test::Fixture f; r.input=f.geometry;
  auto catalog=f.catalog();
  if(analytic) {
    for(unsigned i=analytic==2?0:1;i<2;++i) {
      auto& material=f.materials[i]; material.curve_id=0;
      material.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      material.linear={i?5400.:2700.,i?30000.:20000.}; material.rate={true,8000,8,10000};
    }
    catalog.curve_count=analytic==2?0:1; if(!catalog.curve_count) catalog.curves=nullptr;
  }
  const auto bound=r.binding.Initialize(f.collection());
  EXPECT_EQ(bound.status,fe::ShellBindingStatus::Success)<<bound.message;
  if(bound.status!=fe::ShellBindingStatus::Success) return false;
  r.initial.n=Nodes; r.initial.h=H;
  for(unsigned n=0;n<Nodes;++n) {
    const auto& node=r.binding.nodes()[n];
    r.initial.x[3*n]=node.position.x; r.initial.x[3*n+1]=node.position.y; r.initial.x[3*n+2]=node.position.z;
    r.initial.inverse[n]=1/node.native.mass; r.initial.inverse_inertia[n]=1/node.native.isotropic_inertia;
  }
  const auto state=r.initial.Initialize(r.owner);
  EXPECT_EQ(state.status,fe::NodalStatus::Ok); if(state.status!=fe::NodalStatus::Ok) return false;
  const auto declaration=oracle.Initialize(r.binding,catalog);
  EXPECT_EQ(declaration.status,fe::ShellPlasticityBindingStatus::Success)<<declaration.message;
  if(declaration.status!=fe::ShellPlasticityBindingStatus::Success) return false;
  // Even the temporary catalog disappears after setup. Batch scope and device
  // point parameters must therefore own both declarations and curve samples.
  fe::ShellBatchPlasticityBinding first(oracle);
  q::QephBatchConfig qc; qc.owner=r.owner.accepted(); qc.element_count=1;
  qc.configuration_id=Configuration; qc.qualification_id=Qualification; qc.usage=q::BatchUsage::PrescribedFields;
  const auto qr=r.qeph.InitializeJoined(qc,r.binding,first);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message; if(qr.status!=q::BatchStatus::Success) return false;
  if(mismatch_other_family) f.yq[1]+=1; // T3's own material remains identical.
  fe::ShellBatchPlasticityBinding second;
  const auto second_report=second.Initialize(r.binding,catalog);
  EXPECT_EQ(second_report.status,fe::ShellPlasticityBindingStatus::Success);
  if(second_report.status!=fe::ShellPlasticityBindingStatus::Success) return false;
  t::T3BatchConfig tc; tc.owner=r.owner.accepted(); tc.element_count=1;
  tc.configuration_id=Configuration; tc.qualification_id=Qualification; tc.usage=t::BatchUsage::PrescribedFields;
  fe::ShellBatchPlasticityConfig legacy{38,48,{f.x,f.yt,3},f.materials[1].rate};
  const auto tr=legacy_triangle?r.t3.InitializeJoined(tc,r.binding,legacy):r.t3.InitializeJoined(tc,r.binding,second);
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  for(double& value:f.x) value=-1;
  for(double& value:f.yq) value=-1;
  for(double& value:f.yt) value=-1;
  return tr.status==t::BatchStatus::Success;
}
}
bool InitializeCollection(Rig& r,fe::ShellBatchPlasticityBinding& oracle,
    bool mismatch_other_family,bool legacy_triangle) {
  return InitializeCollectionImpl(r,oracle,mismatch_other_family,legacy_triangle,0);
}
bool InitializeAnalyticCollection(Rig& r,fe::ShellBatchPlasticityBinding& oracle,bool all) {
  return InitializeCollectionImpl(r,oracle,false,false,all?2:1);
}
bool AssembleCollectionForBinding(Rig& r) {
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  const auto begin=r.owner.BeginTrial(&token,&view);
  EXPECT_EQ(begin.status,fe::NodalStatus::Ok); if(begin.status!=fe::NodalStatus::Ok) return false;
  const auto qr=r.qeph.AssembleAccepted(view); const auto tr=r.t3.AssembleAccepted(view);
  EXPECT_EQ(qr.status,q::BatchStatus::Success); EXPECT_EQ(tr.status,t::BatchStatus::Success);
  r.Discard(); return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success;
}
} // namespace resident_plasticity_test
