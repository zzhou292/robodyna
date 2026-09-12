// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include "OriginalFixture.h"
#include "lib_src/assembly/NodalDomainIdentity.h"
namespace beam18_resident_test {
TEST(BeamResidentOriginal, All142NativeConstructorsRetainExactModelAfterCallerLifetime) {
  std::vector<b::ParentInput> source;
  b::Batch batch;
  b::BatchForecast forecast;
  {
    std::vector<fe::NodalDomainNode> nodes;
    std::vector<double> x(std::begin(law44_solid_test::X), std::end(law44_solid_test::X));
    std::vector<double> y(std::begin(law44_solid_test::Y), std::end(law44_solid_test::Y));
    for (unsigned p = 0; p < std::size(beam18_test::original::Cells); ++p) {
      const auto input = beam18_test::original::Input(p);
      b::ParentInput row;
      ASSERT_EQ(b::InitializeReference(input, row.reference), b::Status::Success);
      row.material = beam18_force_test::Material(row.reference);
      row.material.curve = {x.data(), y.data(), static_cast<std::uint32_t>(x.size())};
      source.push_back(row);
      for (unsigned n = 0; n < 3; ++n) {
        const auto id = input.source_node_id[n];
        if (!id) continue;
        const auto position = tl::math::fixed3::Scale(input.position[n], .001);
        const auto found = std::find_if(nodes.begin(), nodes.end(), [id](auto v) { return v.source_id == id; });
        if (found == nodes.end()) nodes.push_back({id, position});
        else ASSERT_TRUE(fe::nodal_domain_detail::SamePosition(found->position, position));
      }
    }
    ASSERT_EQ(source.size(), 142u); ASSERT_EQ(nodes.size(), 147u);
    fe::NodalNodeDomain domain;
    ASSERT_TRUE(domain.Initialize({77, nodes.data(), nodes.size()}));
    b::Model model;
    ASSERT_TRUE(model.Initialize(domain, {77, {source.data(), source.size()}, b::ModelProfile::CircularFourPointLaw44V1}));
    b::BatchConfig config;
    config.owner.owner_id = 37; config.owner.node_count = nodes.size(); config.owner.fixed_dt = 1e-8;
    config.owner.has_rotations = true; config.owner.has_rotation_presence = true;
    config.owner.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
    config.owner.velocity_phase = fe::NodalVelocityPhase::Collocated;
    config.configuration_id = 78; config.qualification_id = 79;
    config.profile = b::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
    // Allocation-only declaration; this fixture never claims a physical owner
    // or asserts its incomplete source domain is a closed vehicle assembly.
    config.cin_attachment_count = 1; config.cin_witness_count = 1;
    ASSERT_TRUE(b::Batch::Forecast(config, model, forecast));
    ASSERT_TRUE(Good(batch.InitializeJoined(config, model)));
  }
  Results results(source.size()); b::BatchDiagnostics diagnostics;
  ASSERT_TRUE(Good(Peer::ReadConstructed(batch, Buffer(results), diagnostics)));
  for (std::size_t p = 0; p < source.size(); ++p) {
    SCOPED_TRACE(source[p].reference.input().source_element_id);
    const auto material = beam18_force_test::Material(source[p].reference);
    const auto native = beam18_force_test::Native(source[p].reference, material, {}, {}, true);
    b::Parent parent; parent.reference = source[p].reference;
    CompareResult(parent, material, results[p], native);
  }
  EXPECT_FALSE(diagnostics.has_completed_interval); EXPECT_EQ(diagnostics.epoch, 0u);
  EXPECT_EQ(diagnostics.parent_count, 142u);
  RecordProperty("resident_device_bytes", std::to_string(forecast.device_bytes));
  RecordProperty("resident_startup_host_bytes", std::to_string(forecast.startup_host_bytes));
}
} // namespace beam18_resident_test
