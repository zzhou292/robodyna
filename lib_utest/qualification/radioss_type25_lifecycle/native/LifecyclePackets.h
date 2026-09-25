// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "LifecycleTables.h"
#include "../../radioss_type25_coefficients/NativeOracle.h"
namespace type25_lifecycle_test::reference {
inline s::NativePairInput Pair(const l::Input& input, int secondary, int local_main,
    std::size_t occurrence) {
  const auto& source = input.source;
  Require(secondary > 0 && std::size_t(secondary) <= source.secondary_count &&
      local_main > 0 && std::size_t(local_main) <= source.main_count, "Native occurrence index out of range");
  const auto& second = source.secondary[secondary-1];
  const auto& node = source.nodes[second.node]; const auto& main = source.mains[local_main-1];
  s::NativePairInput out;
  out.key = {node.source_id, source.generation, std::size_t(secondary-1), main.global_id};
  out.occurrence = occurrence; out.local_main = local_main; out.secondary = Current(input,second.node,false);
  out.secondary_gap = second.gap; out.secondary_coefficient = second.coefficient;
  out.main_coefficient = main.coefficient; out.main_gap_max = main.maximum_gap;
  out.segment_type = main.segment_type; out.initial_contact_flag = second.initial_contact_flag;
  for (unsigned k = 0; k < 4; ++k) {
    const auto& vertex = source.nodes[main.nodes[k]];
    out.main_node_ids[k] = vertex.source_id; out.main_vertices[k] = Current(input,main.nodes[k],false);
    out.main_gap[k] = main.gap[k]; out.normal_slot[k] = main.normal_slot[k]; out.neighbors[k] = main.neighbors[k];
    const auto ref = main.normal_reference[k]; const auto& normal = source.normals[ref-1];
    if (normal.boundary) {
      out.boundary_ids[k] = std::uint64_t(ref);
      for (unsigned b = 0; b < 2; ++b) out.vertex_bisector[k][b] = normal.bisector[b];
    }
  }
  return out;
}
inline s::NativeContinuationInput Continuation(const l::Input& input, const l::Occurrence& item,
    std::size_t ordinal, const int* sliding, int initial_contact) {
  s::NativeContinuationInput out; out.pair = Pair(input,item.secondary,item.local_main,ordinal);
  out.pair.initial_contact_flag = initial_contact;
  out.segment_count = Integer(input.source.main_count);
  const auto& source = input.source; const auto& main = source.mains[item.local_main-1];
  const auto& node = source.nodes[source.secondary[item.secondary-1].node];
  out.secondary_constraint = node.constraint; out.secondary_skew = node.skew;
  for (unsigned k = 0; k < 4; ++k) {
    out.normal_reference[k] = main.normal_reference[k]; out.sliding_reference[k] = sliding[k];
    out.main_constraint[k] = source.nodes[main.nodes[k]].constraint;
    out.main_skew[k] = source.nodes[main.nodes[k]].skew;
  }
  return out;
}
inline s::NativeNewImpactInput NewImpact(const l::Input& input, const l::Occurrence& item,
    std::size_t ordinal, int initial_contact) {
  s::NativeNewImpactInput out; out.pair = Pair(input,item.secondary,item.local_main,ordinal);
  out.pair.initial_contact_flag = initial_contact;
  const auto& source = input.source; const auto& main = source.mains[item.local_main-1];
  out.segment_count = Integer(source.main_count); out.previous_dt = input.step.previous_dt;
  for (unsigned k = 0; k < 4; ++k) out.main_velocity[k] = Current(input,main.nodes[k],true);
  out.secondary_velocity = Current(input,source.secondary[item.secondary-1].node,true);
  if (main.segment_type > 0) {
    const int partner = main.segment_type > out.segment_count ? main.segment_type-out.segment_count : main.segment_type;
    Require(partner > 0 && std::size_t(partner) <= source.main_count && partner != item.local_main,
        "Native opposite main does not identify a distinct supplied row");
    const auto& other = source.mains[partner-1]; out.opposite.local_main = partner;
    out.opposite.global_main = other.global_id;
    for (unsigned k = 0; k < 4; ++k) {
      out.opposite.main_node_ids[k] = source.nodes[other.nodes[k]].source_id;
      out.opposite.normal_slot[k] = other.normal_slot[k]; out.opposite.neighbors[k] = other.neighbors[k];
      const int ref = other.normal_reference[k]; const auto& normal = source.normals[ref-1];
      if (normal.boundary) {
        out.opposite.boundary_ids[k] = std::uint64_t(ref);
        for (unsigned b = 0; b < 2; ++b) out.opposite.vertex_bisector[k][b] = normal.bisector[b];
      }
    }
  }
  return out;
}
inline n::NativeGeometryInput Geometry(const l::Input& input, l::Occurrence& item,
    const n::NativeGeometryHistory& history) {
  Require(item.secondary > 0 && item.local_main == item.cache.local_main,
      "Selected cache/main identity mismatch");
  const auto pair = Pair(input,item.secondary,item.local_main,item.cache.occurrence);
  const auto& marker = history.row.irtlm;
  Require(marker[0] != std::numeric_limits<int>::min() && std::abs(marker[0]) == pair.key.main_segment,
      "Selected native row does not name the kept main");
  const int code = marker[1] % 5; const int sector = code < 0 ? -code : code;
  Require(sector >= 1 && sector <= 4, "Selected native row has no defined sector");
  const auto& cache = item.cache.sector[sector-1];
  Require((cache.defined & s::ClampedBarycentricDefined) != 0,
      "Selected geometry would consume undefined native barycentrics");
  n::NativeGeometryInput out; out.key = pair.key; out.secondary = pair.secondary;
  out.segment_type = pair.segment_type; out.secondary_gap = pair.secondary_gap;
  out.selection_code = marker[1]; out.lb = cache.lb; out.lc = cache.lc;
  out.incoming_stiffness = type25_coefficient_test::Oracle(n::NativePairCoefficientInput{
      pair.main_coefficient,pair.secondary_coefficient,input.profile.minimum_coefficient,
      input.profile.maximum_coefficient}).value;
  for (unsigned k = 0; k < 4; ++k) {
    out.main_node_ids[k] = pair.main_node_ids[k]; out.main_vertices[k] = pair.main_vertices[k];
    out.corner_normal[k] = pair.normal_slot[k]; out.main_gap[k] = pair.main_gap[k];
    out.neighbors[k] = pair.neighbors[k]; out.boundary_ids[k] = pair.boundary_ids[k];
    for (unsigned b = 0; b < 2; ++b) out.vertex_bisector[k][b] = pair.vertex_bisector[k][b];
  }
  item.selected = {true,out.key,item.local_main,sector,out.lb,out.lc,out.incoming_stiffness};
  return out;
}
} // namespace type25_lifecycle_test::reference
