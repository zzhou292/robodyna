// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include <algorithm>
#include <numeric>
#include <stdexcept>
namespace type25_seed_test {
namespace {
unsigned Stage(DirectKind kind) {
  return kind == DirectKind::Truss ? 0 : (kind == DirectKind::Beam18 ? 1 : 2);
}
template<class T, class Less> std::vector<std::size_t> Order(const std::vector<T>& rows, Less less) {
  std::vector<std::size_t> result(rows.size());
  std::iota(result.begin(), result.end(), 0);
  std::stable_sort(result.begin(), result.end(), [&](auto a, auto b) { return less(rows[a], rows[b]); });
  return result;
}
double Coefficient(const Direct& row) {
  if (row.kind == DirectKind::Truss) return row.truss_stiffness;
  if (row.kind == DirectKind::Type45) return 0.; // Source-proved PARGEO2/GEO3, not Kn.
  if (row.kind == DirectKind::Beam18) {
    beam::Reference reference;
    if (beam::InitializeReference(row.beam_input, reference) != beam::Status::Success)
      throw std::runtime_error("Invalid qualification beam packet");
    return reference.native_mass().interface_stiffness;
  }
  n::NativeScalarCoefficient result;
  if (n::EvaluateNativeSpringNodalCoefficient(row.spring, &result) != n::CoefficientStatus::Ok)
    throw std::runtime_error("Invalid qualification spring packet");
  return result.value;
}
}
Prepared PreparePort(const Case& input, SpringOrder order) {
  Prepared result;
  const auto solids = Order(input.solids, [](const auto& a, const auto& b) { return a.eid < b.eid; });
  for (const auto index : solids) {
    const auto& row = input.solids[index];
    n::NativeSolidNodalShares shares;
    if (n::EvaluateNativeSolidNodalShares(row.input, &shares) != n::CoefficientStatus::Ok)
      throw std::runtime_error("Invalid qualification solid packet");
    for (unsigned slot = 0; slot < 8; ++slot) {
      const bool written = shares.defined_raw_slot_mask & (1u << slot);
      // LECTUR initializes all raw slots to+0 before SBULK3; preserve the two
      // untouched Penta occurrences in the native8-slot scatter schedule.
      result.volumes.push_back({row.nodes[slot], written ? shares.volume_share : 0.,
          written ? shares.bulk_volume_share : 0.});
    }
  }
  const auto direct = Order(input.direct, [order](const auto& a, const auto& b) {
    if (Stage(a.kind) != Stage(b.kind)) return Stage(a.kind) < Stage(b.kind);
    if (Stage(a.kind) == 2 && order == SpringOrder::PropertyFamilyNegativeControl && a.kind != b.kind)
      return static_cast<unsigned>(a.kind) < static_cast<unsigned>(b.kind);
    return a.eid < b.eid;
  });
  for (const auto index : direct) {
    const auto& row = input.direct[index];
    const double coefficient = Coefficient(row);
    for (const auto node : row.nodes) result.stiffness.push_back({node, coefficient});
  }
  result.shells = input.shells;
  std::stable_sort(result.shells.begin(), result.shells.end(), [](const auto& a, const auto& b) {
    if (a.layout != b.layout) return a.layout == n::ShellLayout::Quad4;
    return a.source_element_id < b.source_element_id;
  });
  return result;
}
shell::Profile ShellProfile(shell::Population population) {
  shell::Profile profile;
  profile.population = population;
  profile.property_type = 1;
  profile.input_thickness_mode = 0;
  profile.level = 1;
  profile.gap_mode = 1;
  profile.free_edge_gap = 0;
  profile.contact_thickness_update = 0;
  profile.stiffness_scale = 1;
  profile.gap_scale = 1;
  profile.maximum_secondary_gap = 1e30;
  profile.maximum_main_gap = 1e30;
  return profile;
}
Case MixedCase() {
  Case input;
  input.nodes = 16;
  input.solids = {{90,{0,1,2,0,4,5,6,4},{n::SolidNodalKind::Penta6,18.,1.,120.}},
                  {20,{0,1,8,9,4,5,10,11},{n::SolidNodalKind::Hex8,32.,.75,300.}},
                  {50,{0,1,2,2,4,5,6,6},{n::SolidNodalKind::Hex8,12.,1.,80.}}};
  Direct a;
  a.eid = 30; a.nodes = {0,1}; a.kind = DirectKind::Type13;
  a.spring = {n::SpringNodalKind::Type13,1,1,{{3,2},{2,1},{1,1}},2};
  Direct b = a;
  b.eid = 20; b.kind = DirectKind::Type25;
  b.spring = {n::SpringNodalKind::Type25,1,0,{{8,1},{6,1},{0,1}},0};
  Direct joint;
  joint.eid = 25; joint.nodes = {0,4}; joint.kind = DirectKind::Type45; joint.joint_kn = 1e20;
  Direct beam_row;
  beam_row.eid = 400; beam_row.nodes = {0,12}; beam_row.kind = DirectKind::Beam18;
  auto& bi = beam_row.beam_input;
  bi.source_element_id = 400; bi.source_part_id = 4; bi.source_section_id = 4; bi.source_material_id = 4;
  bi.source_node_id[0] = 1; bi.source_node_id[1] = 13; bi.source_node_id[2] = 16;
  bi.position[0] = {0,0,0}; bi.position[1] = {16,0,0}; bi.position[2] = {1,20,0};
  bi.units = beam::WorkingUnits::TonneMillimetreSecond;
  bi.profile = beam::Profile::CircularFourPointStoredZero;
  bi.radius = 4.5; bi.density = 7.89e-9; bi.young = 200000; bi.poisson = .3;
  Direct truss;
  truss.eid = 900; truss.nodes = {0,13}; truss.kind = DirectKind::Truss; truss.truss_stiffness = 17;
  input.direct = {a,joint,beam_row,b,truss};
  input.shells = {{80,n::ShellLayout::Triangle3,{0,2,14,14},300,1,1,1,0},
                  {60,n::ShellLayout::Quad4,{0,1,2,3},100,2,2,2,0},
                  {10,n::ShellLayout::Quad4,{4,5,6,7},20,3,3,3,0}};
  return input;
}
} // namespace type25_seed_test
