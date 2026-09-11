// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

namespace physical_publication_test {
void Fixture::PrepareConstraints() {
  const std::uint64_t members[]{777,9302,9303,9304},plain_ids[]{778,surface_rigid?14u:55u};
  const fe::rigid::PartTopologyPartInput part{200,members,4};
  fe::rigid::PartTopologyInput input;
  input.source_instance_id = 1;
  input.parts = &part;
  input.part_count = 1;
  input.expected_members = members;
  input.expected_member_count = 4;
  input.other_rigid_members = plain_ids;
  input.other_rigid_member_count = 2;
  EXPECT_TRUE(topology.Initialize(input));
  EXPECT_TRUE(parts.Initialize(topology,ledger,{1000,.001}));
  fe::NodalRigidGroupMember group_members[2];
  for (unsigned row = 0; row < 2; ++row) {
    const auto node = domain.Find(plain_ids[row]);
    const auto& value = ledger.nodes()[node].coefficients;
    group_members[row] = {plain_ids[row],node,domain.nodes()[node].position,
        value.mass,value.isotropic_inertia,value.shell.physical_inertia,value.shell.added_inertia};
  }
  const fe::NodalRigidGroupInput group{201,501,group_members,2};
  EXPECT_TRUE(plain.InitializePhysical({29,domain.node_count(),&group,1,{1000,.001}}));
  EXPECT_TRUE(rigid.Initialize(parts,&plain));

  tied::CinAttachmentDeclaration attachment;
  attachment.original_nsv_row = 17;
  attachment.ordered_master_rank = 23;
  attachment.secondary_source_id = 901;
  attachment.master_source = {tied::CinMasterSourceKind::DeclaredShellElement,100,1000};
  attachment.topology = tied::CinMasterTopology::Quad;
  attachment.reference_positions[0] = domain.nodes()[domain.Find(901)].position;
  for (unsigned slot = 0; slot < 4; ++slot) {
    attachment.master_source_ids[slot] = 10+slot;
    attachment.reference_positions[slot+1] = domain.nodes()[domain.Find(10+slot)].position;
  }
  const tied::KinChkSlave slave{901,0,{2,7,7,0,0}};
  std::array<std::int32_t,8192> decode{};
  for (std::size_t code = 0; code < decode.size(); ++code) decode[code] = (code&2) != 0;
  EXPECT_TRUE(tied::PostKinChk({tied::KinChkProfile::NoWallRbeOrCyclic,
      tied::ClassificationPhase::InterfaceTaggedBeforeKinChk,1,881,{&slave,1},
      {decode.data(),decode.size()}},&post));
  EXPECT_TRUE(tied::PrepareCinAttachments(post,domain,{&attachment,1},&cin));
  const std::uint64_t witness_eid[]{100,101,103};
  for (unsigned row = 0; row < 3; ++row) {
    auto& witness = witnesses[row];
    witness.source_element_id = witness_eid[row];
    witness.native_parent_index = row == 1 ? 1 : 0;
    witness.family = tied::cin::WitnessFamily::ShellQuad;
    for (unsigned slot = 0; slot < 4; ++slot) witness.nodes[slot] = domain.Find(10+slot);
  }
}
void Fixture::PrepareMaterials() {
  qbat_catalog_test::Fixture declaration;
  for (unsigned row = 0; row < 2; ++row) {
    declaration.materials[row].curve_id = 0;
    declaration.materials[row].hardening = tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    declaration.materials[row].linear = {10e6,0};
    declaration.materials[row].rate = declaration.materials[2].rate;
  }
  const fe::ShellPlasticityParentInput parents[]{
      {fe::ShellBindingFamily::Qeph,0,100,1000,1000,1000},
      {fe::ShellBindingFamily::Qeph,1,101,1001,1001,1001},
      {fe::ShellBindingFamily::T3,0,102,2000524,2000524,2000524},
      {fe::ShellBindingFamily::Qbat,0,103,2000524,2000524,2000524}};
  EXPECT_EQ(catalog.InitializeFormulations(source.shells,{nullptr,declaration.materials.data(),
      declaration.sections.data(),parents,0,3,3,4}).status,fe::ShellPlasticityBindingStatus::Success);
  fe::ShellFailureParentInput policies[4];
  for (unsigned row = 0; row < 4; ++row) policies[row] = qbat_catalog_test::Failure(parents[row]);
  policies[2].constant.failure_strain=t3_failure;
  for (unsigned row = 0; row < 2; ++row) {
    policies[row].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
    policies[row].constant = {};
    policies[row].tab1.table = {{-1,0,1},1};
  }
  EXPECT_EQ(failure.Initialize(catalog,policies,4).status,fe::ShellPlasticityBindingStatus::Success);
  EXPECT_TRUE(physical.Initialize({&source.shells,&catalog,&failure,nullptr},ledger));
}
} // namespace physical_publication_test
