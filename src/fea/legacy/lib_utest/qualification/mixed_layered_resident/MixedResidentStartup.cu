#include "MixedResidentFixture.h"
#include "lib_src/elements/ShellBatchSectionBinding.h"

namespace mixed_layered_test {
bool Initialize(Rig& r,fe::ShellBatchSectionBinding& catalog,bool other_family_change) {
  plasticity_binding_test::Fixture f;r.input=f.geometry;
  std::array<fe::ShellQephBindingInput,Parents> qs{{f.qeph,f.qeph}};
  std::array<fe::ShellT3BindingInput,Parents> ts{{f.t3,f.t3}};
  qs[1].source_parent_id=703;ts[1].source_parent_id=704;
  // Distinct physical parents retain the five-node fixture and its source IDs.
  // Copy coordinates from their authoritative original parent/local slot.
  qs[1].nodes={0,1,4,3};
  qs[1].reference.position[2]=f.t3.reference.position[1];
  qs[1].reference.node_ids[2]=f.t3.reference.node_ids[1];
  ts[1].nodes={0,4,2};
  ts[1].reference.position[0]=f.qeph.reference.position[0];
  ts[1].reference.node_ids[0]=f.qeph.reference.node_ids[0];
  const auto binding=r.binding.Initialize({qs.data(),ts.data(),Parents,Parents,Nodes});
  EXPECT_EQ(binding.status,fe::ShellBindingStatus::Success);if(binding.status!=fe::ShellBindingStatus::Success)return false;
  r.initial.n=Nodes;r.initial.h=H;
  for(unsigned n=0;n<Nodes;++n) {
    const auto& value=r.binding.nodes()[n];
    r.initial.x[3*n]=value.position.x;r.initial.x[3*n+1]=value.position.y;r.initial.x[3*n+2]=value.position.z;
    r.initial.inverse[n]=1/value.native.mass;r.initial.inverse_inertia[n]=1/value.native.isotropic_inertia;
  }
  const auto owner=r.initial.Initialize(r.owner);EXPECT_EQ(owner.status,fe::NodalStatus::Ok);if(owner.status!=fe::NodalStatus::Ok)return false;
  std::array<fe::ShellSectionMaterialInput,4> materials{{f.materials[0],f.materials[1],f.materials[0],f.materials[1]}};
  for(unsigned i=0;i<2;++i) {
    materials[i].law=fe::ShellSectionLaw::LayeredLaw1Nip3;materials[i].curve_id=0;materials[i].rate={};
  }
  materials[2].material_id=39;materials[2].curve_id=0;
  materials[2].hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
  materials[2].linear={2700,20000};materials[2].rate={true,8000,8,10000};
  materials[3].material_id=40;
  const fe::ShellSectionParentInput parents[]{
    {fe::ShellBindingFamily::T3,0,702,82,40,58},f.parents[1],
    {fe::ShellBindingFamily::Qeph,1,703,83,39,57},
    {fe::ShellBindingFamily::T3,1,704,84,38,58}};
  auto input=f.catalog();input.curves=f.curves.data()+1;input.curve_count=1;
  input.materials=materials.data();input.material_count=4;input.parents=parents;input.parent_count=4;
  const auto bound=catalog.InitializeSections(r.binding,input);EXPECT_EQ(bound.status,fe::ShellSectionBindingStatus::Success)<<bound.message;
  if(bound.status!=fe::ShellSectionBindingStatus::Success)return false;
  q::QephBatchConfig qc;qc.owner=r.owner.accepted();qc.element_count=Parents;
  qc.configuration_id=Configuration;qc.qualification_id=Qualification;qc.usage=q::BatchUsage::PrescribedFields;
  t::T3BatchConfig tc;tc.owner=r.owner.accepted();tc.element_count=Parents;
  tc.configuration_id=Configuration;tc.qualification_id=Qualification;tc.usage=t::BatchUsage::PrescribedFields;
  fe::ShellBatchSectionBinding copy(catalog);
  const auto qr=r.qeph.InitializeJoined(qc,r.binding,copy);EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  fe::ShellBatchSectionBinding other;
  if(other_family_change) {
    materials[2].linear.initial_yield_pa+=1; // T3's own laws/parameters remain identical.
    const auto report=other.InitializeSections(r.binding,input);
    EXPECT_EQ(report.status,fe::ShellSectionBindingStatus::Success);
    if(report.status!=fe::ShellSectionBindingStatus::Success)return false;
  }
  const auto tr=r.t3.InitializeJoined(tc,r.binding,other_family_change?other:copy);EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success;
}
} // namespace mixed_layered_test
