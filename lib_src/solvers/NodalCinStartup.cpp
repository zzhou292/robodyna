// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinStorage.h"
#include "../constraints/NodalRigidGroupModel.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace tl::fea::nodal_detail {
namespace {
namespace cin = constraints::tied_shell::cin;
bool SameBits(double a, double b) noexcept { return std::memcmp(&a, &b, sizeof(a)) == 0; }
bool Contains(const cin::ActiveWitness& witness, std::uint32_t node) noexcept {
  for (const auto candidate : witness.nodes) if (candidate == node) return true;
  return false;
}
bool ValidWitness(const cin::ActiveWitness& witness, const cin::StageRow& row, std::size_t n) noexcept {
  if (!witness.source_element_id) return false;
  const bool triangle = witness.family == cin::WitnessFamily::ShellTriangle;
  if (!triangle && witness.family != cin::WitnessFamily::ShellQuad) return false;
  if (triangle && witness.nodes[2] != witness.nodes[3]) return false;
  const unsigned distinct = triangle ? 3 : 4;
  for (const auto node : witness.nodes) if (node >= n) return false;
  for (unsigned i = 0; i < distinct; ++i) {
    for (unsigned j = i+1; j < distinct; ++j) {
      if (witness.nodes[i] == witness.nodes[j]) return false;
    }
  }
  // Native CHK2MSR3NB tests physical-node containment, not EID equality,
  // orientation or INCOQ material selection. Positive exact shell witnesses
  // also establish positive incidence at every selected master node.
  for (const auto node : row.masters) if (!Contains(witness, node)) return false;
  return true;
}
}

NodalReport ForecastCinStorage(const NodalCinStartup& input, const NodalStateConfig& config,
    CinLayout& output) noexcept {
  if (!input.model || !input.model->prepared() || !input.model->domain() ||
      input.model->domain()->node_count() != config.node_count || !input.qualification_id) {
    return {NodalStatus::InvalidInput, "CIN needs a complete immutable domain and named qualification"};
  }
  if (config.temporal_scheme != NodalTemporalScheme::StaggeredHalfKickStart) {
    return {NodalStatus::UnsupportedTemporalScheme, "CIN requires the existing staggered owner"};
  }
  CinLayout next;
  if (!next.Initialize(config.node_count, input.model->rows().count, input.witness_count,
      input.limits, sizeof(CinStorage))) {
    return {NodalStatus::ResourceLimit, "CIN count or complete optional payload exceeds limits"};
  }
  const auto source = input.model->forecast();
  for (const auto bytes : {source.model_payload_bytes, source.domain_payload_bytes, source.post_kinchk_payload_bytes}) {
    if (bytes > input.limits.max_host_bytes-next.host_bytes) {
      return {NodalStatus::ResourceLimit, "Retained CIN source backing exceeds host limits"};
    }
    next.host_bytes += bytes;
  }
  output = next;
  return {NodalStatus::Ok, "CIN storage forecast prepared"};
}

NodalReport PrepareCinStorage(const NodalCinStartup& input, const NodalStateConfig& config,
    HostNodalKinematicsView kinematics, const double* inverse_mass, const NodalDofConfig& dofs,
    const NodalRigidGroupModel* groups, const CinLayout& layout, std::unique_ptr<CinStorage>& output) {
  if (!input.mass || !input.inertia || !input.witness_ranges || !input.witnesses) {
    return {NodalStatus::InvalidInput, "CIN coefficient and complete witness inputs are mandatory"};
  }
  // All finite raw values/reciprocal/source-bit checks precede optional arrays.
  const auto domain = input.model->domain()->nodes();
  for (std::size_t i = 0; i < config.node_count; ++i) {
    if (!std::isfinite(input.mass[i]) || input.mass[i] < 0 ||
        !std::isfinite(input.inertia[i]) || input.inertia[i] < 0) {
      return {NodalStatus::InvalidInput, "CIN current M/J must be finite and nonnegative", std::uint32_t(i)};
    }
    const double xyz[] = {domain[i].position.x, domain[i].position.y, domain[i].position.z};
    for (unsigned a = 0; a < 3; ++a) {
      if (!SameBits(xyz[a], kinematics.position_xyz[3*i+a])) {
        return {NodalStatus::InvalidInput, "CIN owner coordinates differ from exact domain source bits", std::uint32_t(i)};
      }
    }
  }
  const auto source_rows = input.model->rows();
  std::size_t next_offset = 0;
  for (std::size_t r = 0; r < source_rows.count; ++r) {
    const auto& source = source_rows.data[r];
    cin::StageRow row;
    row.secondary = source.secondary_domain_node;
    std::copy(source.master_domain_nodes.begin(), source.master_domain_nodes.end(), row.masters);
    row.witnesses = input.witness_ranges[r];
    if (row.witnesses.offset != next_offset || !row.witnesses.count || row.witnesses.count > 4 ||
        row.witnesses.count > input.witness_count-next_offset) {
      return {NodalStatus::InvalidInput, "CIN witness ranges must cover the complete source-ordered input", row.secondary};
    }
    if (dofs.translation_fixed_bits[row.secondary] || dofs.rotation_fixed[row.secondary]) {
      return {NodalStatus::InvalidInput, "CIN dependent DOFs must be free", row.secondary};
    }
    for (const auto node : row.masters) {
      if (dofs.translation_fixed_bits[node] || dofs.rotation_fixed[node]) {
        return {NodalStatus::InvalidInput, "First CIN profile requires free ordinary masters", node};
      }
    }
    for (std::size_t w = next_offset; w < next_offset+row.witnesses.count; ++w) {
      if (!ValidWitness(input.witnesses[w], row, config.node_count)) {
        return {NodalStatus::InvalidInput, "CIN activity witness does not contain the native four slots", row.secondary};
      }
    }
    next_offset += row.witnesses.count;
  }
  if (next_offset != input.witness_count) return {NodalStatus::InvalidInput, "CIN has trailing witness declarations"};
  auto next = std::make_unique<CinStorage>();
  next->source = *input.model;
  next->layout = layout;
  next->qualification_id = input.qualification_id;
  next->rows.resize(source_rows.count);
  next->dependent.resize(config.node_count, 0);
  next->witnesses.assign(input.witnesses, input.witnesses+input.witness_count);
  next->first_witness.resize(input.witness_count);
  // Fresh vectors normally reserve exactly the requested count. Charge their
  // actual backing capacities as well, before any device allocation.
  auto actual_host_bytes = layout.host_bytes;
  const auto extra = [&](std::size_t capacity, std::size_t size, std::size_t width) {
    if (capacity < size || (capacity-size) > (input.limits.max_host_bytes-actual_host_bytes)/width) return false;
    actual_host_bytes += (capacity-size)*width;
    return true;
  };
  if (!extra(next->rows.capacity(), next->rows.size(), sizeof(cin::StageRow)) ||
      !extra(next->dependent.capacity(), next->dependent.size(), sizeof(std::uint8_t)) ||
      !extra(next->witnesses.capacity(), next->witnesses.size(), sizeof(cin::ActiveWitness)) ||
      !extra(next->first_witness.capacity(), next->first_witness.size(), sizeof(std::uint32_t))) {
    return {NodalStatus::ResourceLimit, "Actual CIN host capacities exceed the declared payload limit"};
  }
  next->layout.host_bytes = actual_host_bytes;
  util::SourceIdentityIndex<16> witness_index;
  witness_index.Prepare(input.witness_count, [&](std::size_t i) { return input.witnesses[i].source_element_id; });
  for (std::size_t i = 0; i < input.witness_count; ++i) {
    const auto first = witness_index.First(input.witnesses[i].source_element_id);
    next->first_witness[i] = std::uint32_t(first);
    const auto& a = input.witnesses[first];
    const auto& b = input.witnesses[i];
    if (a.native_parent_index != b.native_parent_index || a.family != b.family ||
        !std::equal(a.nodes, a.nodes+4, b.nodes)) {
      return {NodalStatus::InvalidInput, "Repeated CIN source witness has conflicting native association"};
    }
  }
  for (std::size_t r = 0; r < source_rows.count; ++r) {
    const auto& source = source_rows.data[r];
    auto& row = next->rows[r];
    row.secondary = source.secondary_domain_node;
    std::copy(source.master_domain_nodes.begin(), source.master_domain_nodes.end(), row.masters);
    row.witnesses = input.witness_ranges[r];
    if (next->dependent[row.secondary]) return {NodalStatus::InvalidInput, "Duplicate CIN dependent", row.secondary};
    next->dependent[row.secondary] = 1;
  }
  for (const auto& row : next->rows) {
    for (const auto node : row.masters) {
      if (next->dependent[node]) return {NodalStatus::InvalidInput, "CIN hierarchy/secondary-master overlap is not admitted", node};
    }
  }
  for (std::size_t i = 0; i < config.node_count; ++i) {
    if (next->dependent[i]) {
      if (inverse_mass[i] != 0 || dofs.inverse_inertia[i] != 0) {
        return {NodalStatus::InvalidInput, "CIN dependents have no conventional inverse or kick", std::uint32_t(i)};
      }
    } else if ((dofs.translation_fixed_bits[i] != 7 &&
          (!input.mass[i] || inverse_mass[i] != 1/input.mass[i])) ||
        (!dofs.rotation_fixed[i] && (!input.inertia[i] || dofs.inverse_inertia[i] != 1/input.inertia[i]))) {
      return {NodalStatus::InvalidInput, "Independent raw M/J and supplied inverse association differ", std::uint32_t(i)};
    }
  }
  if (groups) {
    // Reuse the already budgeted membership array as a temporary role index.
    // Bit zero always means dependent; bit one is removed before publication.
    for (const auto& row : next->rows) {
      for (const auto node : row.masters) next->dependent[node] |= 2;
    }
    for (std::size_t i = 0; i < groups->member_count(); ++i) {
      const auto node = groups->members()[i].global_node;
      if (node >= config.node_count || next->dependent[node]) {
        return {NodalStatus::InvalidInput, "CIN master/dependent intersects an actual rigid member", std::uint32_t(node)};
      }
    }
    for (auto& role : next->dependent) role &= 1;
  }
  output = std::move(next);
  return {NodalStatus::Ok, "CIN startup source, witness and coefficient association prepared"};
}
} // namespace tl::fea::nodal_detail
