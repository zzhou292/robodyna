// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NativeOracle.h"
#include "NativeAbi.h"
#include <cmath>
#include <limits>
#include <stdexcept>
namespace type25_lifecycle_test::reference {
inline void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}
inline int Integer(std::uint64_t value) {
  Require(value <= std::uint64_t(std::numeric_limits<int>::max()), "Native integer table overflow");
  return static_cast<int>(value);
}
inline void Finite(double value) { Require(std::isfinite(value), "Nonfinite lifecycle oracle operand"); }
inline void Finite(n::Vector v) { Finite(v.x); Finite(v.y); Finite(v.z); }
inline void Finite(n::StoredNormal v) { Finite(v.x); Finite(v.y); Finite(v.z); }
inline std::vector<int> Offsets(const l::Csr& csr, std::size_t count, std::size_t upper) {
  Require(csr.offset_count == count + 1 && csr.offsets && csr.entry_count <= 8192 &&
      (!csr.entry_count || csr.entries), "Native CSR extent is not bounded");
  Require(csr.offsets[0] == 0 && csr.offsets[count] == csr.entry_count, "Native CSR endpoints disagree");
  std::vector<int> result(count + 1);
  for (std::size_t i = 0; i <= count; ++i) {
    Require(csr.offsets[i] <= csr.entry_count && (!i || csr.offsets[i-1] <= csr.offsets[i]),
        "Native CSR offsets are not monotonic");
    result[i] = Integer(csr.offsets[i]);
  }
  for (std::size_t i = 0; i < csr.entry_count; ++i)
    Require(csr.entries[i] >= 1 && csr.entries[i] <= upper, "Native CSR main reference is out of range");
  return result;
}
inline std::vector<int> Entries(const l::Csr& csr) {
  std::vector<int> result(csr.entry_count);
  for (std::size_t i = 0; i < csr.entry_count; ++i) result[i] = Integer(csr.entries[i]);
  return result;
}
inline n::Vector Current(const l::Input& input, std::size_t index, bool velocity) {
  const auto view = velocity ? input.current.velocities : input.current.positions;
  Require(view.valid() && view.node_count == input.source.node_count && index < view.node_count,
      "Lifecycle reference kinematics view is invalid");
  const auto raw = view.at(static_cast<std::uint32_t>(index));
  n::Vector result{raw.x,raw.y,raw.z}; Finite(result);
  if (input.current.units == l::KinematicsUnits::Native) return result;
  Require(input.current.units == l::KinematicsUnits::Si, "Unspecified lifecycle kinematics units");
  const auto units = input.current.native_units;
  Finite(units.length_m); Finite(units.mass_kg); Finite(units.time_s);
  Require(units.length_m > 0 && units.mass_kg > 0 && units.time_s > 0, "Invalid native unit scale");
  const double factor = velocity ? units.length_m / units.time_s : units.length_m;
  Finite(factor); Require(factor > 0, "Unrepresentable native kinematics unit");
  result = {result.x/factor,result.y/factor,result.z/factor}; Finite(result); return result;
}
struct Tables {
  std::vector<int> main_nodes, main_global, main_role, normal_refs, secondary_nodes, node_ids;
  std::vector<int> normal_offsets, normal_mains, removal_offsets, removed_mains;
  std::vector<double> main_stiffness;
  explicit Tables(const l::Input& in) {
    const auto& source = in.source;
    Require(source.main_count && source.main_count <= 32 && source.mains &&
        source.node_count && source.node_count <= 256 && source.nodes &&
        source.normal_count && source.normal_count <= 256 && source.normals &&
        source.secondary_count <= 128 && (!source.secondary_count || source.secondary) &&
        in.spatial_count <= 4096 && (!in.spatial_count || in.spatial) && source.generation &&
        in.accepted_row_count == source.secondary_count &&
        (!in.accepted_row_count || in.accepted_rows), "Lifecycle reference table bounds exceeded");
    main_nodes.resize(4*source.main_count); main_global.resize(source.main_count);
    main_role.resize(source.main_count); normal_refs.resize(4*source.main_count);
    main_stiffness.resize(source.main_count); secondary_nodes.resize(source.secondary_count);
    node_ids.resize(source.node_count);
    for (std::size_t i = 0; i < source.node_count; ++i) {
      const auto& node = source.nodes[i]; Require(node.source_id != 0, "Zero native node identity");
      node_ids[i] = Integer(node.source_id); Current(in,i,false); Current(in,i,true);
      for (std::size_t j = 0; j < i; ++j)
        Require(source.nodes[j].source_id != node.source_id, "Native node map contains duplicate identities");
    }
    for (std::size_t i = 0; i < source.normal_count; ++i)
      for (auto normal : source.normals[i].bisector) Finite(normal);
    for (std::size_t i = 0; i < source.main_count; ++i) {
      const auto& main = source.mains[i]; Require(main.global_id > 0, "Nonpositive native global main");
      Require(std::int64_t(main.segment_type) >= -2*std::int64_t(source.main_count) &&
          std::int64_t(main.segment_type) <= 2*std::int64_t(source.main_count), "Native role out of range");
      for (std::size_t j = 0; j < i; ++j)
        Require(source.mains[j].global_id != main.global_id, "Duplicate native global main identity");
      main_global[i] = main.global_id; main_role[i] = main.segment_type;
      Finite(main.coefficient); main_stiffness[i] = main.coefficient; Finite(main.maximum_gap);
      for (unsigned k = 0; k < 4; ++k) {
        Require(main.nodes[k] < source.node_count && main.normal_reference[k] > 0 &&
            std::size_t(main.normal_reference[k]) <= source.normal_count, "Native main table index out of range");
        main_nodes[4*i+k] = Integer(main.nodes[k] + 1); normal_refs[4*i+k] = main.normal_reference[k];
        Finite(main.gap[k]); Finite(main.normal_slot[k]);
      }
    }
    for (std::size_t i = 0; i < source.secondary_count; ++i) {
      const auto& secondary = source.secondary[i];
      Require(secondary.node < source.node_count, "Native secondary node index out of range");
      secondary_nodes[i] = Integer(secondary.node + 1); Finite(secondary.coefficient); Finite(secondary.gap);
      const auto& row = in.accepted_rows[i];
      Require(row.secondary_source_id == source.nodes[secondary.node].source_id &&
          row.generation == source.generation && row.row.irtlm[0] != std::numeric_limits<int>::min(),
          "Native accepted row identity/marker mismatch");
    }
    normal_offsets = Offsets(source.normal_to_main, source.normal_count, source.main_count);
    normal_mains = Entries(source.normal_to_main);
    removal_offsets = Offsets(source.removed_main_by_secondary, source.secondary_count, source.main_count);
    removed_mains = Entries(source.removed_main_by_secondary);
  }
};
struct Rows {
  std::vector<int> markers, sliding;
  std::vector<double> metrics, penetration, stiffness, friction;
  explicit Rows(std::size_t count) : markers(4*count), sliding(4*count), metrics(2*count),
      penetration(5*count), stiffness(2*count), friction(6*count) {}
  void Store(std::size_t i, const n::NativeContactRow& row) {
    for (unsigned k = 0; k < 4; ++k) markers[4*i+k] = row.irtlm[k];
    for (unsigned k = 0; k < 2; ++k) metrics[2*i+k] = row.selection_metric[k];
    const auto& h = row.history;
    penetration[5*i] = h.normal.staged_penetration; penetration[5*i+1] = h.normal.previous_penetration;
    penetration[5*i+2] = h.normal.damping_half_force; penetration[5*i+3] = row.penetration_auxiliary;
    penetration[5*i+4] = row.penetration_offset;
    stiffness[2*i] = h.normal.staged_stiffness; stiffness[2*i+1] = h.normal.previous_stiffness;
    friction[6*i] = h.staged_force.x; friction[6*i+1] = h.staged_force.y; friction[6*i+2] = h.staged_force.z;
    friction[6*i+3] = h.previous_force.x; friction[6*i+4] = h.previous_force.y; friction[6*i+5] = h.previous_force.z;
  }
  void Load(std::size_t i, n::NativeContactRow& row) const {
    for (unsigned k = 0; k < 4; ++k) row.irtlm[k] = markers[4*i+k];
    for (unsigned k = 0; k < 2; ++k) row.selection_metric[k] = metrics[2*i+k];
    auto& h = row.history;
    h.normal.staged_penetration = penetration[5*i]; h.normal.previous_penetration = penetration[5*i+1];
    h.normal.damping_half_force = penetration[5*i+2]; row.penetration_auxiliary = penetration[5*i+3];
    row.penetration_offset = penetration[5*i+4]; h.normal.staged_stiffness = stiffness[2*i];
    h.normal.previous_stiffness = stiffness[2*i+1];
    h.staged_force = {friction[6*i],friction[6*i+1],friction[6*i+2]};
    h.previous_force = {friction[6*i+3],friction[6*i+4],friction[6*i+5]};
  }
};
} // namespace type25_lifecycle_test::reference
