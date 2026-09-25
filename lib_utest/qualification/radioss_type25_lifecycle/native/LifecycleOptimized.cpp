// SPDX-License-Identifier: AGPL-3.0-or-later
#include "LifecycleStages.h"
#include "LifecyclePackets.h"
namespace type25_lifecycle_test::reference {
void FilterRaw(const l::Input& input, const Tables& table, Rows& rows, OracleResult& output) {
  const auto& source = input.source; const auto prefix = output.occurrences.size();
  const auto capacity = prefix+input.spatial_count;
  std::vector<double> kinematics(6*source.node_count), main_gap(source.main_count);
  std::vector<double> secondary_stiffness(source.secondary_count), secondary_gap(source.secondary_count);
  std::vector<int> contact(source.secondary_count), raw_n(input.spatial_count), raw_e(input.spatial_count);
  std::vector<int> candidate_n(capacity), candidate_e(capacity), ordinals(capacity);
  for (std::size_t i = 0; i < source.node_count; ++i) {
    for (unsigned v = 0; v < 2; ++v) {
      const auto value = Current(input,i,v != 0); const auto offset = 3*(i+v*source.node_count);
      kinematics[offset] = value.x; kinematics[offset+1] = value.y; kinematics[offset+2] = value.z;
    }
  }
  for (std::size_t i = 0; i < source.main_count; ++i) main_gap[i] = source.mains[i].maximum_gap;
  for (std::size_t i = 0; i < source.secondary_count; ++i) {
    secondary_stiffness[i] = source.secondary[i].coefficient; secondary_gap[i] = source.secondary[i].gap;
    contact[i] = output.rows[i].initial_contact_flag;
  }
  for (std::size_t i = 0; i < input.spatial_count; ++i) {
    raw_n[i] = input.spatial[i].secondary; raw_e[i] = input.spatial[i].local_main;
  }
  for (std::size_t i = 0; i < prefix; ++i) {
    candidate_n[i] = output.occurrences[i].secondary; candidate_e[i] = output.occurrences[i].local_main;
  }
  const int counts[]{Integer(source.node_count),Integer(source.main_count),Integer(source.secondary_count),
      Integer(input.spatial_count),Integer(capacity)};
  int used = Integer(prefix), required = 0;
  rd_lifecycle_optcd(counts,&input.profile.optcd_response_precision,&input.step.previous_dt,
      kinematics.data(),table.main_nodes.data(),table.main_global.data(),table.main_role.data(),
      table.main_stiffness.data(),main_gap.data(),table.secondary_nodes.data(),secondary_stiffness.data(),
      secondary_gap.data(),rows.markers.data(),rows.metrics.data(),rows.friction.data(),rows.penetration.data(),
      rows.stiffness.data(),raw_n.data(),raw_e.data(),candidate_n.data(),candidate_e.data(),&used,contact.data(),ordinals.data(),&required);
  Require(required == used && used >= Integer(prefix) && std::size_t(used) <= capacity, "Native OPTCD exceeded its complete raw roster bound");
  for (std::size_t i = 0; i < input.spatial_count; ++i)
    Require(raw_n[i] == input.spatial[i].secondary && raw_e[i] == input.spatial[i].local_main,
        "Native OPTCD did not restore retained raw inventory signs");
  for (std::size_t i = 0; i < prefix; ++i)
    Require(candidate_n[i] == output.occurrences[i].secondary && candidate_e[i] == output.occurrences[i].local_main,
        "Native OPTCD modified its retained prefix");
  std::size_t previous = 0;
  for (std::size_t i = prefix; i < std::size_t(used); ++i) {
    Require(ordinals[i] > 0 && std::size_t(ordinals[i]) <= input.spatial_count &&
        (i == prefix || std::size_t(ordinals[i]) > previous), "Native OPTCD observation changed input order");
    previous = std::size_t(ordinals[i]); const auto ordinal = previous-1;
    const auto& original = input.spatial[ordinal];
    Require(candidate_n[i] == original.secondary && candidate_e[i] == original.local_main,
        "Native OPTCD output disagrees with its observed original ordinal");
    l::Occurrence item; item.secondary = original.secondary; item.local_main = original.local_main;
    item.origin = l::Origin::Spatial; item.source_ordinal = ordinal;
    const auto pair = Pair(input,item.secondary,item.local_main,i);
    item.cache.key = pair.key; item.cache.local_main = pair.local_main; item.cache.occurrence = i;
    output.occurrences.push_back(item);
    ++output.rows[item.secondary-1].optimized_count;
  }
  const int count = Integer(source.secondary_count);
  rd_lifecycle_release_main(&count,rows.markers.data());
  for (std::size_t i = 0; i < output.rows.size(); ++i) {
    rows.Load(i,output.rows[i].history.row); output.rows[i].initial_contact_flag = contact[i];
  }
}
} // namespace type25_lifecycle_test::reference
