// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solids/resident/AssemblyValues.h"
#include "lib_src/elements/solids/resident/AssemblyRanges.h"
#include "lib_src/elements/solids/resident/Storage.h"
#include "lib_utils/OrderedNodeIncidence.h"
#include <stdexcept>
#include <vector>
namespace solid_parallel_test {
namespace fe = tl::fea;
namespace d = fe::solids::batch_detail;
namespace s = fe::solids;
// Deliberately private arithmetic packets, not source/physical owner authority.
// Public owner/real-history coverage remains in extended_solid_resident.
struct Packet {
  tl::util::HostArena arena;
  d::ArenaLayout layout;
  std::size_t nodes;
  explicit Packet(std::size_t node_count = 64, std::size_t parents_per_family = 2)
      : nodes(node_count) {
    s::BatchConfig config;
    config.owner.node_count = nodes;
    // The existing solid arena includes a complete owner-proof footprint and
    // requires at least one CIN attachment even for these private arithmetic
    // packets. This count creates no live attachment or physical authority.
    config.cin_attachment_count = 1;
    config.cin_witness_count = 1;
    d::Counts counts{parents_per_family, parents_per_family, parents_per_family,
        1, 1, 1, parents_per_family, parents_per_family, 1, 1};
    if (!d::MakeLayout(counts, config, layout))
      throw std::runtime_error("Packet layout");
    if (!arena.Initialize(layout.bytes))
      throw std::runtime_error("Packet allocation");
    auto* state = arena.Construct<d::Storage>(layout.header);
    *state = d::RebasedHeader(arena.data(), layout);
    state->config = config;
    Fill<d::Traits18>(layout.solid18, 0);
    Fill<d::Traits24>(layout.solid24, 1);
    Fill<d::Traits6z>(layout.solid6z, 2);
    Fill<d::Traits18Law44>(layout.solid18_law44, 3);
    Fill<d::Traits18Law90>(layout.solid18_law90, 4);
    if (!arena.Construct<std::uint32_t>(layout.assembly_offsets) ||
        !arena.Construct<std::uint32_t>(layout.assembly_incidence) ||
        !arena.Construct<d::AssemblyNode>(layout.assembly_nodes))
      throw std::runtime_error("Packet typed storage");
    Rebuild();
  }
  d::Storage& State() { return *tl::util::ArenaPointer<d::Storage>(arena.data(), layout.header); }
  template<class Traits> void Fill(const d::FamilyLayout& family, unsigned kind) {
    auto* parents = arena.Construct<typename Traits::Parent>(family.parents);
    auto* values = arena.Construct<d::State<Traits>>(family.slab[0]);
    if (!parents || !values) throw std::runtime_error("Packet family");
    for (std::size_t p = 0; p < family.parents.count; ++p) {
      values[p].cache.stiffness.translation_n_m = 1.+kind+.125*p;
      for (unsigned slot = 0; slot < Traits::nodes; ++slot) {
        parents[p].domain_nodes[slot] = (3*p+kind+slot)%nodes;
        const double x = (slot%2 ? -1. : 1.)*(p+1);
        values[p].cache.rhs_force_n[slot] = {x, .25*x, -.125*x};
      }
      if constexpr (std::is_same_v<Traits, d::Traits18Law44>) {
        parents[p].domain_nodes[5] = parents[p].domain_nodes[4];
        parents[p].domain_nodes[7] = parents[p].domain_nodes[6];
      }
    }
  }
  void Rebuild() {
    auto& state = State();
    if (!tl::util::BuildOrderedNodeIncidence<1>(layout.assembly_incidence.count, nodes,
        [&](std::size_t ordinal, unsigned) {
          d::AssemblyOccurrence value;
          return d::ReadAssemblyOccurrence<false>(state, 0, ordinal, value) ? value.node : SIZE_MAX;
        }, state.assembly.offsets, nodes+1, state.assembly.incidence,
        layout.assembly_incidence.count)) throw std::runtime_error("Packet incidence");
  }
};
} // namespace solid_parallel_test
