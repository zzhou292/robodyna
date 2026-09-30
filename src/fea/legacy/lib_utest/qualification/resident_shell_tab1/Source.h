#pragma once
#include "../shell_placement_force/PlacementForceFixture.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"
#include <array>

namespace resident_tab1_test {
namespace fe = tl::fea;
namespace storage = fe::shell_batch_plasticity_detail;
namespace placed = placement_force_test;
using Placement = fe::ShellReferencePlacement;
constexpr std::size_t Parents = 4, Nodes = 7 * Parents;
// Complete source-shaped collection: LAW1/None, LAW44/None, constant all-point
// failure, and the original analytic filtered-zero-C TAB1 glass composition.
// Coordinates are synthetic qualification packets, not an imported vehicle.
struct Source {
  std::array<fe::ShellQephBindingInput, Parents> q;
  std::array<fe::ShellT3BindingInput, Parents> t;
  std::array<fe::ShellPlasticityMaterialInput, 2> materials;
  fe::ShellPlasticitySectionInput section{501, .00228, 3};
  std::array<fe::ShellPlasticityParentInput, 2 * Parents> parents;
  std::array<fe::ShellFailureParentInput, 2 * Parents> failures;
  explicit Source(Placement plane = Placement::Centered) {
    materials[0].material_id = 301;
    materials[0].young_pa = 70e9;
    materials[0].poisson_ratio = .22;
    materials[0].density_kg_m3 = 2500;
    materials[0].law = fe::ShellSectionLaw::LayeredLaw1Nip3;
    materials[1] = materials[0];
    materials[1].material_id = 302;
    materials[1].law = fe::ShellSectionLaw::LayeredLaw44Nip3;
    materials[1].hardening = tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    materials[1].linear = {30e6, 1e9};
    materials[1].rate = {true, 0, 1, 10000, tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
    for (unsigned e = 0; e < Parents; ++e) {
      const auto selected = e == Parents - 1 ? plane : Placement::Centered;
      q[e].reference = placed::Fixture<placed::Q>(selected).reference.input;
      t[e].reference = placed::Fixture<placed::T>(selected).reference.input;
      q[e].source_parent_id = 1001 + 2 * e;
      t[e].source_parent_id = 1002 + 2 * e;
      for (unsigned n = 0; n < 4; ++n) {
        q[e].nodes[n] = 7 * e + n;
        q[e].reference.node_ids[n] = static_cast<std::uint32_t>(q[e].nodes[n] + 2001);
      }
      for (unsigned n = 0; n < 3; ++n) {
        t[e].nodes[n] = 7 * e + 4 + n;
        t[e].reference.node_ids[n] = t[e].nodes[n] + 2001;
      }
      const auto mid = e ? 302u : 301u;
      parents[2 * e] = {fe::ShellBindingFamily::T3, e, t[e].source_parent_id, 102 + 2 * e, mid, 501};
      parents[2 * e + 1] = {fe::ShellBindingFamily::Qeph, e, q[e].source_parent_id, 101 + 2 * e, mid, 501};
      for (unsigned i = 2 * e; i < 2 * e + 2; ++i) {
        failures[i].source = parents[i];
        if (e == 2) {
          failures[i].policy = fe::ShellFailurePolicy::ConstantAllPoints;
          failures[i].constant.failure_strain = .015;
        } else if (e == 3) {
          failures[i].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
          failures[i].tab1 = {tab1_test::Table()};
        }
      }
    }
  }
  bool Prepare(fe::ShellBatchBinding& binding, fe::ShellBatchPlasticityBinding& catalog) const {
    const auto b = binding.Initialize({q.data(), t.data(), Parents, Parents, Nodes});
    EXPECT_EQ(b.status, fe::ShellBindingStatus::Success) << b.message;
    if (b.status != fe::ShellBindingStatus::Success) return false;
    const fe::ShellBatchPlasticityBindingInput input{
        nullptr, materials.data(), &section, parents.data(), 0, materials.size(), 1, parents.size()};
    const auto c = catalog.InitializeSections(binding, input);
    EXPECT_EQ(c.status, fe::ShellPlasticityBindingStatus::Success) << c.message;
    return c.status == fe::ShellPlasticityBindingStatus::Success;
  }
};
} // namespace resident_tab1_test
