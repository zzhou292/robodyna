// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
namespace beam18_resident_test {
OwnerFixture::OwnerFixture() {
  auto& f = mechanics;
  const std::uint64_t nodes[3][2]{{778, 9305}, {9302, 9306}, {9305, 9306}};
  b::ParentInput input[3];
  for (unsigned p = 0; p < 3; ++p) {
    auto source = beam18_force_test::Reference().input();
    source.source_element_id = 32000 + p;
    source.source_part_id = 32100;
    source.source_section_id = 32200;
    source.source_material_id = 32300;
    for (unsigned n = 0; n < 2; ++n) {
      source.source_node_id[n] = nodes[p][n];
      source.position[n] = f.domain.nodes()[f.domain.Find(nodes[p][n])].position;
    }
    source.source_node_id[2] = 0;
    source.position[2] = {};
    EXPECT_EQ(b::InitializeReference(source, input[p].reference), b::Status::Success);
    input[p].material = beam18_force_test::Material(input[p].reference);
  }
  const auto prepared = model.Initialize(f.domain, {f.domain.source_instance_id(), {input, 3}, b::ModelProfile::CircularFourPointLaw44V1});
  EXPECT_TRUE(prepared) << prepared.message;
  if (!prepared) return;
  EXPECT_TRUE(beams.Initialize(model));
  EXPECT_TRUE(ledger.InitializeWithBeams({{{&f.shells, &f.springs}, &f.point, &f.solids}, &beams}));
  EXPECT_TRUE(parts.Initialize(f.topology, ledger, {1000, .001}));
  std::array<fe::NodalRigidGroupMember, 2> members;
  for (unsigned k = 0; k < 2; ++k) {
    const auto node = f.domain.Find(f.plain_ids[k]);
    const auto& c = ledger.nodes()[node].coefficients;
    members[k] = {f.plain_ids[k], node, f.domain.nodes()[node].position, c.mass, c.isotropic_inertia,
        c.shell.physical_inertia, c.shell.added_inertia, c.beam18.isotropic_inertia};
  }
  const fe::NodalRigidGroupInput group{200, 501, members.data(), members.size()};
  const auto plain_report = plain.InitializeNativeTotal({29, f.domain.node_count(), &group, 1, {1000, .001}});
  EXPECT_TRUE(plain_report) << plain_report.message;
  EXPECT_TRUE(binding.Initialize(parts, &plain));
  for (std::size_t i = 0; i < f.m.size(); ++i) {
    f.m[i] = ledger.nodes()[i].coefficients.mass;
    f.j[i] = ledger.nodes()[i].coefficients.isotropic_inertia;
    const bool member = binding.FindMember(i);
    f.fixed[i] = !f.m[i] && !member ? 7 : 0;
    f.present[i] = f.j[i] > 0 || member;
  }
  f.DependentInverses(true);
}
b::BatchConfig OwnerFixture::Configuration() const {
  b::BatchConfig c;
  c.owner.owner_id = 19;
  c.owner.node_count = mechanics.domain.node_count();
  c.owner.fixed_dt = 1e-8;
  c.owner.has_rotations = true;
  c.owner.has_rotation_presence = true;
  c.owner.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
  c.owner.velocity_phase = fe::NodalVelocityPhase::Collocated;
  c.owner.rigid_groups = {1, binding.groups().size(), binding.members().size(), parts.roots().size(),
      binding.plain_source_instance_id()};
  c.configuration_id = 902;
  c.qualification_id = mechanics.Cin().qualification_id;
  c.profile = b::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
  c.cin_attachment_count = mechanics.ranges.size();
  c.cin_witness_count = mechanics.witnesses.size();
  return c;
}
} // namespace beam18_resident_test
