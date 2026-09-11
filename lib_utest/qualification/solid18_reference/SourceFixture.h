// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "source_fixture/YarisSolid18SourceFixture.h"

namespace solid18_test {
namespace original = yaris_solid18_source_fixture;
inline constexpr unsigned SourceCount = sizeof(original::solids)/sizeof(original::solids[0]);
static_assert(SourceCount == 908);
static_assert(sizeof(original::nodes)/sizeof(original::nodes[0]) == 3672);
inline s::ReferenceInput Source(unsigned index, bool working_units = false) {
  const auto& row = original::solids[index];
  s::ReferenceInput input;
  input.source_element_id = row.id;
  input.source_part_id = original::part_id;
  input.source_material_id = original::part_id;
  input.source_section_id = original::part_id;
  input.density_kg_m3 = working_units ? 1.07e-9 : 1.07e-9*1e12;
  for (unsigned n = 0; n < 8; ++n) {
    const auto& node = original::nodes[row.node[n]];
    input.source_node_id[n] = node.id;
    const auto* x = working_units ? node.native : node.position_m;
    input.position_m[n] = {x[0],x[1],x[2]};
  }
  return input;
}
}  // namespace solid18_test
