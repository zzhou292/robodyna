// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
namespace extended_resident_test {
OwnerFixture::OwnerFixture(bool analytic44,bool controls,s::control::UnitScale units,bool collapsed,bool packet_pair,unsigned controlled_packet_pairs):controlled(controls) {
  auto& f=Mechanics();
  s::Input18Law44 rear;
  auto source=f.source.a;
  source.profile=fe::solid18::law44::Profile();
  source.source_element_id=19100;source.source_part_id=2000016;source.source_material_id=2000016;
  source.density_kg_m3=7.89e-9*1e12;
  if (analytic44) {
    source.source_part_id = source.source_section_id = source.source_material_id = 2000945;
    source.density_kg_m3 = 1.95e-9 * 1e12;
  }
  source.source_node_id[5]=source.source_node_id[4];source.position_m[5]=source.position_m[4];
  source.source_node_id[7]=source.source_node_id[6];source.position_m[7]=source.position_m[6];
  EXPECT_EQ(fe::solid18::law44::InitializeReference(source,rear.reference),fe::solid18::Status::Success);
  const tl::material::law44::solid::Material steel{50e9,.3,source.density_kg_m3,8000,8,10000,
    tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond};
  if (analytic44) {
    auto material = steel; material.young_pa = 1e9;
    EXPECT_EQ(tl::material::law44::solid::PrepareMat024Analytic(material, 1000, 20, 10, rear.material),
      tl::material::law44::solid::Status::Ok);
  } else {
    EXPECT_EQ(tl::material::law44::solid::Prepare(steel,{rear_x,rear_y,3},rear.material),
      tl::material::law44::solid::Status::Ok);
  }
  s::Input18Law90 foam;
  source=f.source.a;
  source.profile=fe::solid18::total_strain::Law90Profile();
  source.source_element_id=19101;source.source_part_id=2000006;source.source_material_id=2000006;
  source.density_kg_m3=772;
  EXPECT_EQ(fe::solid18::total_strain::InitializeReference90(source,foam.reference),fe::solid18::Status::Success);
  foam_input.hysteresis=0;
  if(controls)foam_input.tension_cutoff_pa=15e6;
  foam_input.density_kg_m3=772;foam_input.card_young_pa=72e6;foam_input.contact_modulus_pa=4e6;
  EXPECT_EQ(tl::material::law90::PrepareSI(foam_input,{foam_x,foam_y,3},foam.material),tl::material::law90::Status::Ok);
  s::ModelInput input{1,{&legacy.input18,1},{&legacy.input24,1},{&legacy.input6z,1},
    {&rear,1},{&foam,1},s::ModelProfile::ExtendedLaw44Law90};
  if(controls||collapsed) {
    auto h=legacy.input24.reference.input();
    if(controls)h.profile.working_length=units.length_m==1?fe::solid24::WorkingLengthUnit::Metre:fe::solid24::WorkingLengthUnit::Millimetre;
    if(collapsed){h.profile.connectivity=fe::solid24::ConnectivityProfile::CollapsedTopEdges;
    h.source_node_id[5]=h.source_node_id[4];h.position_m[5]=h.position_m[4];
    h.source_node_id[7]=h.source_node_id[6];h.position_m[7]=h.position_m[6];}
    EXPECT_EQ(fe::solid24::InitializeReference(h,legacy.input24.reference),fe::solid24::Status::Success);
  }
  std::vector<s::Input24> h24{legacy.input24};
  std::vector<s::Input18Law90> foams{foam};
  if(controls) {
    // Eight independent H24 packets followed by eight LAW90 packets force all
    // eight workers to reuse storage across types; four-worker replay uses the
    // identical authenticated packet roster and element count.
    if(packet_pair){auto p=legacy.input24;auto h=f.source.b;h.source_element_id=19102;
      h.profile.working_length=units.length_m==1?fe::solid24::WorkingLengthUnit::Metre:fe::solid24::WorkingLengthUnit::Millimetre;
      EXPECT_EQ(fe::solid24::InitializeReference(h,p.reference),fe::solid24::Status::Success);h24.push_back(p);}
    for(unsigned i=1;i<controlled_packet_pairs;++i){
      auto p=legacy.input24;auto h=f.source.b;h.source_element_id=19110+i;
      h.profile.working_length=units.length_m==1?fe::solid24::WorkingLengthUnit::Metre:fe::solid24::WorkingLengthUnit::Millimetre;
      EXPECT_EQ(fe::solid24::InitializeReference(h,p.reference),fe::solid24::Status::Success);h24.push_back(p);
      auto other=foam;auto ref=foam.reference.input();ref.source_element_id=19200+i;
      EXPECT_EQ(fe::solid18::total_strain::InitializeReference90(ref,other.reference),fe::solid18::Status::Success);foams.push_back(other);
    }
    input.solid24={h24.data(),h24.size()};input.solid18_law90={foams.data(),foams.size()};
  }
  std::vector<s::control::SourceParent> rows;
  std::vector<s::control::NativePacket> packets;
  std::vector<std::uint64_t> packet_members;
  s::control::NativePartition partition{0,0,2*controlled_packet_pairs+3,0,2*controlled_packet_pairs+3+unsigned(packet_pair)};
  if(controls) {
    auto row=[&](const auto& ref,unsigned ctl){const auto& a=ref.input();
      rows.push_back({a.source_element_id,a.source_part_id,a.source_section_id,a.source_material_id,a.source_section_id,ctl});
      packet_members.push_back(a.source_element_id);};
    auto add=[&](const auto& ref,s::Family family,unsigned ctl) {
      const auto& a=ref.input();const auto p=packet_members.size();
      packets.push_back({packets.size()+1,p,p,1,family,a.source_material_id,a.source_section_id,ctl});row(ref,ctl);
    };
    add(h24[0].reference,s::Family::Solid24,1);
    if(packet_pair){row(h24[1].reference,1);++packets.back().member_count;}
    for(unsigned i=1+unsigned(packet_pair);i<h24.size();++i)add(h24[i].reference,s::Family::Solid24,1);
    for(const auto& p:foams)add(p.reference,s::Family::Solid18Law90,1);
    add(legacy.input18.reference,s::Family::Solid18,0);
    add(rear.reference,s::Family::Solid18Law44,0);
    add(legacy.input6z.reference,s::Family::Solid6z,0);
    auto& c=input.controls;c.profile=s::control::Profile::SourceDeclared;c.source_instance_id=input.source_instance_id;
    c.units=units;c.native_nvsiz=128;c.compiled_mvsiz=129;
    c.parents={rows.data(),rows.size()};c.packets={packets.data(),packets.size()};
    c.partitions={&partition,1};c.ordered_element_ids={packet_members.data(),packet_members.size()};
  }
  const auto initialized=model.Initialize(f.domain,input);
  EXPECT_TRUE(initialized)<<initialized.message;
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
