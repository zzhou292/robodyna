// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
namespace extended_resident_test {
OwnerFixture::OwnerFixture() {
  auto& f=Mechanics();
  s::Input18Law44 rear;
  auto source=f.source.a;
  source.profile=fe::solid18::law44::Profile();
  source.source_element_id=19100;source.source_part_id=2000016;source.source_material_id=2000016;
  source.density_kg_m3=7.89e-9*1e12;
  source.source_node_id[5]=source.source_node_id[4];source.position_m[5]=source.position_m[4];
  source.source_node_id[7]=source.source_node_id[6];source.position_m[7]=source.position_m[6];
  EXPECT_EQ(fe::solid18::law44::InitializeReference(source,rear.reference),fe::solid18::Status::Success);
  const tl::material::law44::solid::Material steel{50e9,.3,source.density_kg_m3,8000,8,10000,
    tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond};
  EXPECT_EQ(tl::material::law44::solid::Prepare(steel,{rear_x,rear_y,3},rear.material),
    tl::material::law44::solid::Status::Ok);
  s::Input18Law90 foam;
  source=f.source.a;
  source.profile=fe::solid18::total_strain::Law90Profile();
  source.source_element_id=19101;source.source_part_id=2000006;source.source_material_id=2000006;
  source.density_kg_m3=772;
  EXPECT_EQ(fe::solid18::total_strain::InitializeReference90(source,foam.reference),fe::solid18::Status::Success);
  foam_input.hysteresis=0;
  foam_input.density_kg_m3=772;foam_input.card_young_pa=72e6;foam_input.contact_modulus_pa=4e6;
  EXPECT_EQ(tl::material::law90::PrepareSI(foam_input,{foam_x,foam_y,3},foam.material),tl::material::law90::Status::Ok);
  s::ModelInput input{1,{&legacy.input18,1},{&legacy.input24,1},{&legacy.input6z,1},
    {&rear,1},{&foam,1},s::ModelProfile::ExtendedLaw44Law90};
  EXPECT_TRUE(model.Initialize(f.domain,input));
  EXPECT_TRUE(ledger.InitializeWithExtendedSolids({{&f.shells,&f.springs},&f.point,model.contributions()}));
  EXPECT_TRUE(parts.Initialize(f.topology,ledger,{1000,.001}));
  std::array<fe::NodalRigidGroupMember,2> members;
  for(unsigned k=0;k<2;++k) {
    const auto node=f.domain.Find(f.plain_ids[k]);const auto& c=ledger.nodes()[node].coefficients;
    members[k]={f.plain_ids[k],node,f.domain.nodes()[node].position,c.mass,c.isotropic_inertia,
      c.shell.physical_inertia,c.shell.added_inertia};
  }
  const fe::NodalRigidGroupInput group{200,501,members.data(),members.size()};
  EXPECT_TRUE(plain.InitializePhysical({29,f.domain.node_count(),&group,1,{1000,.001}}));
  EXPECT_TRUE(binding.Initialize(parts,&plain));
  for(std::size_t i=0;i<f.m.size();++i) {
    f.m[i]=ledger.nodes()[i].coefficients.mass;
    f.j[i]=ledger.nodes()[i].coefficients.isotropic_inertia;
    const bool member=binding.FindMember(i);
    f.fixed[i]=!f.m[i]&&!member?7:0;
    f.present[i]=f.j[i]>0||member;
  }
  f.DependentInverses(true);
}
} // namespace extended_resident_test
