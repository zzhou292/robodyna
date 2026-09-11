// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qt_mapped_test {
void MakeSkinFixture(Fixture& f,bool triangle) {
  // Explicit synthetic PART source: both native families have a skin parent.
  // The second QEPH and QBAT remain constitutive on the same physical nodes.
  // The ordinary fixture separately exercises coexistence with plain groups.
  f.skin.emplace();
  auto& scope=*f.skin;
  const std::uint64_t members[]{10,11,12,13,14,777,9302,9303,9304};
  const fe::rigid::PartTopologyPartInput part{200,members,9};
  fe::rigid::PartTopologyInput source;
  source.source_instance_id=1;
  source.parts=&part;
  source.part_count=1;
  source.expected_members=members;
  source.expected_member_count=9;
  fe::rigid::NodalRigidPartTopology topology;
  ASSERT_TRUE(topology.Initialize(source));
  ASSERT_TRUE(scope.parts.Initialize(topology,f.ledger,{1000,.001}));
  ASSERT_TRUE(scope.rigid.Initialize(scope.parts));
  qbat_catalog_test::Fixture declarations;
  auto& first=declarations.materials[0];
  first.curve_id=0;
  if (triangle) {
    first.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    first.linear={10e6,0};
    first.rate=declarations.materials[2].rate;
  } else {
    first.law=fe::ShellSectionLaw::RigidSkin;
    declarations.sections[0].through_thickness_points=0;
    declarations.sections[0].formulation=fe::ShellSectionFormulation::Nonconstitutive;
  }
  auto& glass=declarations.materials[1];
  glass.curve_id=0;
  glass.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
  glass.linear={10e6,0};
  glass.rate=declarations.materials[2].rate;
  std::vector<fe::ShellPlasticityMaterialInput> materials(declarations.materials.begin(),declarations.materials.end());
  std::vector<fe::ShellPlasticitySectionInput> sections(declarations.sections.begin(),declarations.sections.end());
  if (triangle) {
    fe::ShellPlasticityMaterialInput skin;
    skin.material_id=3000;
    skin.young_pa=materials[2].young_pa;
    skin.poisson_ratio=materials[2].poisson_ratio;
    skin.density_kg_m3=materials[2].density_kg_m3;
    skin.law=fe::ShellSectionLaw::RigidSkin;
    materials.push_back(skin);
    sections.push_back({3000,sections[2].thickness_m,0,fe::ShellSectionFormulation::Nonconstitutive});
  }
  fe::ShellPlasticityParentInput parents[]{
      {fe::ShellBindingFamily::Qeph,0,100,200,1000,1000},
      {fe::ShellBindingFamily::Qeph,1,101,1001,1001,1001},
      {fe::ShellBindingFamily::T3,0,102,2000524,2000524,2000524},
      {fe::ShellBindingFamily::Qbat,0,103,2000524,2000524,2000524}};
  if (triangle) {
    parents[0].source_part_id=1000;
    parents[2]={fe::ShellBindingFamily::T3,0,102,200,3000,3000};
  }
  const fe::ShellBatchPlasticityBindingInput input{nullptr,materials.data(),
      sections.data(),parents,0,materials.size(),sections.size(),4};
  const auto catalog=scope.catalog.InitializeExecutionCatalog(f.mechanics.source.shells,input);
  ASSERT_EQ(catalog.status,fe::ShellPlasticityBindingStatus::Success)<<catalog.message;
  fe::ShellFailureParentInput rows[4];
  for (unsigned i=0;i<4;++i) rows[i].source=parents[i];
  rows[1].policy=fe::ShellFailurePolicy::Tab1AnyPoint;
  rows[1].tab1.table={{-1,0,1},1};
  rows[3]=qbat_catalog_test::Failure(parents[3]);
  if (triangle) { rows[0]=rows[1]; rows[0].source=parents[0]; }
  else rows[2]=qbat_catalog_test::Failure(parents[2]);
  ASSERT_EQ(scope.failure.InitializeExecution(scope.catalog,rows,4).status,fe::ShellPlasticityBindingStatus::Success);
  fe::ShellExecutionBinding execution;
  ASSERT_EQ(execution.Initialize(scope.catalog,f.ledger,scope.rigid).status,fe::ShellPlasticityBindingStatus::Success);
  ASSERT_TRUE(scope.physical.InitializeExecution({&f.mechanics.source.shells,&scope.catalog,&scope.failure,nullptr},
      f.ledger,execution));
  for (auto id:members) {
    const auto node=f.mechanics.domain.Find(id);
    f.mechanics.present[node]=1;
    f.mechanics.fixed[node]=0;
  }
  f.mechanics.DependentInverses(true);
}
} // namespace qt_mapped_test
