// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

namespace rigid_assembly_owner_test {
Fixture::Fixture(bool intersection,bool physical_plain,double point_mass_source) {
  if (intersection) part_ids[3]=cin_source.declarations[0].master_source_ids[0];
  if (physical_plain) {
    source.nodes.push_back({778,{.06,-.01,.003}});
    plain_ids[0]=778;
  }
  source.nodes.insert(source.nodes.end(),cin_source.nodes.begin(),cin_source.nodes.end());
  EXPECT_TRUE(domain.Initialize({1,source.nodes.data(),source.nodes.size()}));
  EXPECT_TRUE(shells.Initialize(source.shells,domain));
  fe::solid18::Reference solid;
  EXPECT_EQ(fe::solid18::InitializeReference(source.a,solid),fe::solid18::Status::Success);
  EXPECT_TRUE(solids.Initialize(domain,{1,&solid,1}));
  zero_mass = domain.Find(777);
  const fe::ElementMassSource masses[]{{18000,777,zero_mass,0},
      {18001,778,physical_plain?domain.Find(778):SIZE_MAX,point_mass_source}};
  EXPECT_TRUE(point.Initialize(domain,{1,1000,masses,physical_plain?2u:1u}));
  // Real TYPE25 value producers provide the independent CIN patch coefficients.
  // Their recurrence is not a participant in this prescribed-load owner test.
  std::vector<fe::type25::ConnectionInput> connectors;
  for (const auto& node : cin_source.nodes) {
    const auto i = domain.Find(node.source_id);
    auto anchor = domain.Find(12);
    const auto anchor_position = domain.nodes()[anchor].position;
    if (node.position.x == anchor_position.x && node.position.y == anchor_position.y &&
        node.position.z == anchor_position.z) anchor = domain.Find(13);
    fe::type25::ConnectionInput connection;
    connection.source_element_id = 19000+connectors.size();
    connection.global_node[0] = anchor;
    connection.global_node[1] = i;
    connection.source_node_id[0] = domain.nodes()[anchor].source_id;
    connection.source_node_id[1] = node.source_id;
    connection.position[0] = domain.nodes()[anchor].position;
    connection.position[1] = node.position;
    connectors.push_back(connection);
  }
  fe::type25::ModelInput spring_input;
  spring_input.source_instance_id = 1;
  spring_input.global_node_count = domain.node_count();
  spring_input.source_units = {1,1,1};
  spring_input.properties = &source.spring_property;
  spring_input.property_count = 1;
  spring_input.connections = connectors.data();
  spring_input.connection_count = connectors.size();
  const auto spring_report = springs.Initialize(spring_input);
  EXPECT_TRUE(spring_report) << spring_report.message;
  EXPECT_TRUE(ledger.InitializeWithSolids({{&shells,&springs},&point,&solids}));
  const r::PartTopologyPartInput part{200,part_ids.data(),part_ids.size()};
  r::PartTopologyInput top;
  top.source_instance_id = 1;
  top.parts = &part;
  top.part_count = 1;
  top.expected_members = part_ids.data();
  top.expected_member_count = part_ids.size();
  top.other_rigid_members = plain_ids.data();
  top.other_rigid_member_count = plain_ids.size();
  EXPECT_TRUE(topology.Initialize(top));
  EXPECT_TRUE(parts.Initialize(topology,ledger,{1000,.001}));
  std::array<fe::NodalRigidGroupMember,2> members;
  for (unsigned k=0;k<2;++k) {
    const auto n = domain.Find(plain_ids[k]);
    const auto& c = ledger.nodes()[n].coefficients;
    members[k] = {plain_ids[k],n,domain.nodes()[n].position,c.mass,c.isotropic_inertia,
      c.shell.physical_inertia,c.shell.added_inertia};
  }
  const fe::NodalRigidGroupInput group{200,501,members.data(),members.size()};
  const fe::NodalRigidGroupModelInput plain_input{29,domain.node_count(),&group,1,{1000,.001}};
  EXPECT_TRUE(physical_plain?plain.InitializePhysical(plain_input):plain.Initialize(plain_input));
  EXPECT_TRUE(binding.Initialize(parts,&plain));
  auto post_input = cin_source.PostInput();
  post_input.source_instance_id = 1;
  EXPECT_TRUE(tied::PostKinChk(post_input,&post));
  EXPECT_TRUE(tied::PrepareCinAttachments(post,domain,cin_source.Input(),&cin_model));
  for (std::size_t k=0;k<cin_model.rows().count;++k) {
    const auto& row = cin_model.rows().data[k];
    ranges.push_back({std::uint32_t(k),1});
    cin::ActiveWitness witness;
    witness.source_element_id = row.master_source.element_id;
    witness.native_parent_index = k;
    witness.family = k ? cin::WitnessFamily::ShellTriangle : cin::WitnessFamily::ShellQuad;
    std::copy(row.master_domain_nodes.begin(),row.master_domain_nodes.end(),witness.nodes);
    witnesses.push_back(witness);
  }
  const auto n = domain.node_count();
  x.resize(3*n);v.resize(3*n);w.resize(3*n);q.resize(4*n);
  m.resize(n);j.resize(n);im.resize(n);ij.resize(n);
  fixed.resize(n);rotation_fixed.resize(n);present.resize(n);
  for (std::size_t i=0;i<n;++i) {
    const auto p = domain.nodes()[i].position;
    x[3*i]=p.x;x[3*i+1]=p.y;x[3*i+2]=p.z;
    q[4*i]=1;
    m[i]=ledger.nodes()[i].coefficients.mass;
    j[i]=ledger.nodes()[i].coefficients.isotropic_inertia;
    const bool member = binding.FindMember(i);
    fixed[i]=!m[i]&&!member?7:0;
    present[i]=j[i]>0||member;
  }
  ordinary = domain.Find(9305);
  DependentInverses(false);
}
void Fixture::DependentInverses(bool use_cin) {
  for (std::size_t i=0;i<m.size();++i) {
    im[i]=fixed[i]==7||m[i]==0?0:1/m[i];
    ij[i]=!present[i]||j[i]==0?0:1/j[i];
  }
  if (use_cin) for (std::size_t k=0;k<cin_model.rows().count;++k) {
    const auto& row=cin_model.rows().data[k];
    im[row.secondary_domain_node]=0;
    ij[row.secondary_domain_node]=0;
  }
}
fe::NodalStateConfig Fixture::Config() const {
  fe::NodalStateConfig c;
  c.node_count=m.size();
  c.fixed_dt=H;
  c.max_device_bytes=4u<<20;
  c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  c.rigid_limits=fe::NodalRigidOwnerLimits::VehicleAssembly();
  return c;
}
} // namespace rigid_assembly_owner_test
