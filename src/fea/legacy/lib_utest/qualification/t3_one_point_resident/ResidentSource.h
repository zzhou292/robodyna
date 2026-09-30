#pragma once
#include "../resident_shell_failure/FailureResidentSource.h"
#include "../t3_one_point/Fixture.h"

namespace t3_one_point_resident_test {
namespace fe = tl::fea;
namespace pure = t3_one_point_test;
constexpr unsigned Parents = 2, Nodes = 5;
// Test collection: three synthetic neighbors surround the exact ordered
// EID2357656 triangle. This is not source admission of the whole midlayer PID.
struct Source {
  resident_failure_test::Source base;
  std::array<fe::ShellPlasticitySectionInput, 3> sections;
  explicit Source(bool one_point = true, double failure_strain = 2.5) {
    pure::Fixture original;
    const auto& ref = original.reference.input;
    const auto a = ref.position[0];
    const auto u = pure::Sub(ref.position[1], a);
    const auto v = pure::Sub(ref.position[2], a);
    const fe::t3::Vec3 x[]{a, pure::Add(a, pure::Add(pure::Scale(u, .5), pure::Scale(v, -.5))),
        ref.position[2], pure::Add(a, pure::Add(pure::Scale(u, -.5), pure::Scale(v, .75))), ref.position[1]};
    const std::uint64_t ids[]{ref.node_ids[0], 1001, ref.node_ids[2], 1003, ref.node_ids[1]};
    for (auto& parent : base.q) {
      for (unsigned local = 0; local < 4; ++local) {
        const auto n = parent.nodes[local];
        parent.reference.position[local] = x[n];
        parent.reference.node_ids[local] = ids[n];
      }
    }
    for (auto& parent : base.t) {
      for (unsigned local = 0; local < 3; ++local) {
        const auto n = parent.nodes[local];
        parent.reference.position[local] = x[n];
        parent.reference.node_ids[local] = ids[n];
      }
    }
    base.t[1].nodes = {0, 4, 2};
    base.t[1].reference = ref;
    base.t[1].source_parent_id = 2357656;
    auto& material = base.materials[3];
    material.young_pa = ref.young_modulus;
    material.poisson_ratio = ref.poisson_ratio;
    material.density_kg_m3 = ref.density;
    material.curve_id = 0;
    material.hardening = tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    material.linear = original.material.linear;
    material.rate = original.material.rate;
    sections = {{base.seed.sections[0], base.seed.sections[1],
        {59, ref.thickness, one_point ? 1u : 3u,
         one_point ? fe::ShellSectionFormulation::OneThicknessPoint : fe::ShellSectionFormulation::LayeredNip3}}};
    base.parents[3] = {fe::ShellBindingFamily::T3, 1, 2357656, 2000524, material.material_id, 59};
    base.failures[3].source = base.parents[3];
    base.failures[3].constant.failure_strain = failure_strain;
    // QEPH1 uses an analytic declaration; the other two synthetic neighbors
    // are elastic. No declaration now uses the original seed's table.
  }
  fe::ShellBatchPlasticityBindingInput Input() const {
    return {nullptr, base.materials.data(), sections.data(), base.parents.data(),
        0, base.materials.size(), sections.size(), base.parents.size()};
  }
  bool Prepare(fe::ShellBatchBinding& geometry, fe::ShellBatchPlasticityBinding& catalog,
      fe::ShellBatchFailureBinding& failure) const {
    auto report = geometry.Initialize({base.q.data(), base.t.data(), Parents, Parents, Nodes});
    EXPECT_EQ(report.status, fe::ShellBindingStatus::Success) << report.message;
    if (report.status != fe::ShellBindingStatus::Success) return false;
    const auto material = catalog.InitializeSections(geometry, Input());
    EXPECT_EQ(material.status, fe::ShellPlasticityBindingStatus::Success) << material.message;
    if (material.status != fe::ShellPlasticityBindingStatus::Success) return false;
    const auto failed = failure.Initialize(catalog, base.failures.data(), base.failures.size());
    EXPECT_EQ(failed.status, fe::ShellPlasticityBindingStatus::Success) << failed.message;
    return failed.status == fe::ShellPlasticityBindingStatus::Success;
  }
};
} // namespace t3_one_point_resident_test
