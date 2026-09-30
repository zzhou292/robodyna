// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "../solid_common/source_fixture/YarisRubberSourceFixture.h"
namespace solid6z_test {
namespace original=yaris_rubber_source_fixture;
inline constexpr unsigned SourceCount=sizeof(original::solids)/sizeof(original::solids[0]);
static_assert(SourceCount==1504);
static_assert(sizeof(original::nodes)/sizeof(original::nodes[0])==2308);
inline bool Source(unsigned index,s::ReferenceInput& input,bool working=false) {
  const auto& row=original::solids[index];
  std::uint64_t raw[8];
  for (unsigned n=0;n<8;++n) raw[n]=original::nodes[row.node[n]].id;
  s::CollapsedBrickTopology map;
  if (s::MapCollapsedTopEdges(raw,map)!=s::Status::Success) return false;
  input.source_element_id=row.id; input.source_part_id=row.part_id;
  input.source_section_id=row.part_id; input.source_material_id=row.part_id;
  input.density_kg_m3=working?1.98e-9:1.98e-9*1e12;
  for (unsigned n=0;n<6;++n) {
    const auto& node=original::nodes[row.node[map.six_to_raw[n]]];
    input.source_node_id[n]=node.id;
    const auto* x=working?node.native:node.position_m;
    input.position_m[n]={x[0],x[1],x[2]};
  }
  return true;
}
}  // namespace solid6z_test
