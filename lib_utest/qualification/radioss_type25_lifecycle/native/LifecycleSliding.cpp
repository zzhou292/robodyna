// SPDX-License-Identifier: AGPL-3.0-or-later
#include "LifecycleStages.h"
#include "LifecyclePackets.h"
namespace type25_lifecycle_test::reference {
void PrepareSliding(const l::Input& input, const Tables& table, Rows& rows,
    std::size_t retained, OracleResult& output) {
  const int nsn = Integer(output.rows.size());
  if (!retained) return;
  const int counts[]{nsn,Integer(input.source.main_count),Integer(input.source.normal_count),Integer(retained)};
  std::vector<int> candidates_n(retained), candidates_e(retained), far(4*retained);
  std::vector<double> penetration(4*retained), lb(4*retained), lc(4*retained);
  for (std::size_t i = 0; i < retained; ++i) {
    const auto& item = output.occurrences[i]; const auto row = std::size_t(item.secondary-1);
    candidates_n[i] = item.secondary; candidates_e[i] = item.local_main;
    const auto& main = input.source.mains[item.local_main-1];
    if (rows.markers[4*row+1] <= 0 && main.nodes[2] != main.nodes[3]) {
      const auto sector = -std::int64_t(rows.markers[4*row+1])/5;
      Require(sector >= 1 && sector <= 4, "Native PREP_SLID1 would read an undefined Q4 sector");
    }
    for (unsigned k = 0; k < 4; ++k) {
      const auto& cache = item.cache.sector[k];
      Require((cache.defined & (s::FarDefined|s::PenetrationDefined)) ==
          (s::FarDefined|s::PenetrationDefined), "PREP_SLID1 needs the retained native cache");
      far[4*i+k] = cache.far; penetration[4*i+k] = cache.penetration;
      lb[4*i+k] = cache.lb; lc[4*i+k] = cache.lc;
    }
  }
  rd_lifecycle_prepare1(counts,candidates_n.data(),candidates_e.data(),table.main_nodes.data(),
      table.main_global.data(),table.normal_refs.data(),rows.markers.data(),rows.metrics.data(),
      far.data(),penetration.data(),lb.data(),lc.data(),rows.sliding.data());
  for (std::size_t i = 0; i < output.rows.size(); ++i) {
    rows.Load(i,output.rows[i].history.row);
    for (unsigned k = 0; k < 4; ++k) output.rows[i].sliding_reference[k] = rows.sliding[4*i+k];
  }
}
void AppendSliding(const l::Input& input, const Tables& table, Rows& rows, OracleResult& output) {
  const std::size_t prefix = output.occurrences.size();
  // ITAGM admits each main at most once per secondary. This finite reference
  // ceiling counts the complete append without truncating the native traversal.
  const std::size_t capacity = prefix + input.source.secondary_count*input.source.main_count;
  std::vector<int> candidates_n(capacity), candidates_e(capacity);
  for (std::size_t i = 0; i < prefix; ++i) {
    candidates_n[i] = output.occurrences[i].secondary; candidates_e[i] = output.occurrences[i].local_main;
  }
  for (int ref : rows.sliding)
    Require(ref >= 0 && std::size_t(ref) <= input.source.normal_count, "Native sliding reference is not indexable");
  const int counts[]{Integer(input.source.secondary_count),Integer(input.source.main_count),
      Integer(input.source.normal_count),Integer(capacity),Integer(table.normal_mains.size()),Integer(table.removed_mains.size())};
  int used = Integer(prefix), found = 0;
  rd_lifecycle_expand(counts,table.main_nodes.data(),table.main_global.data(),table.main_role.data(),
      table.normal_refs.data(),table.secondary_nodes.data(),rows.sliding.data(),rows.markers.data(),
      table.main_stiffness.data(),table.normal_offsets.data(),table.normal_mains.data(),
      table.removal_offsets.data(),table.removed_mains.data(),&input.profile.neighbor_removal,
      candidates_n.data(),candidates_e.data(),&used,&found);
  Require(found >= 0 && used == Integer(prefix)+found && std::size_t(used) <= capacity,
      "Native sliding append exceeded its proved unique-main bound");
  for (std::size_t i = 0; i < prefix; ++i)
    Require(candidates_n[i] == output.occurrences[i].secondary && candidates_e[i] == output.occurrences[i].local_main,
        "Native sliding append modified the original prefix");
  for (std::size_t i = prefix; i < std::size_t(used); ++i) {
    l::Occurrence item; item.secondary = candidates_n[i]; item.local_main = candidates_e[i];
    const auto pair = Pair(input,item.secondary,item.local_main,i);
    item.origin = l::Origin::Sliding; item.source_ordinal = output.rows[item.secondary-1].sliding_count++;
    item.cache.key = pair.key; item.cache.local_main = pair.local_main; item.cache.occurrence = i;
    output.occurrences.push_back(item);
  }
}
std::vector<int> Membership(const l::Input& input, const Tables& table, Rows& rows,
    const OracleResult& output, int phase) {
  const auto size = output.occurrences.size(); std::vector<int> candidates_n(size), candidates_e(size), indices(size);
  for (std::size_t i = 0; i < size; ++i) {
    candidates_n[i] = output.occurrences[i].secondary; candidates_e[i] = output.occurrences[i].local_main;
  }
  const int counts[]{Integer(input.source.secondary_count),Integer(input.source.main_count),Integer(size)};
  int count = 0;
  rd_lifecycle_membership(counts,&phase,candidates_n.data(),candidates_e.data(),table.main_global.data(),
      rows.markers.data(),indices.data(),&count);
  Require(count >= 0 && std::size_t(count) <= size, "Native membership count is outside its roster");
  indices.resize(std::size_t(count)); return indices;
}
} // namespace type25_lifecycle_test::reference
