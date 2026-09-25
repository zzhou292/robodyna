// SPDX-License-Identifier: AGPL-3.0-or-later
#include "LifecycleStages.h"
#include "../../radioss_type25_friction/NativeOracle.h"
namespace type25_lifecycle_test::reference {
void Keep(const l::Input& input, const Tables& table, Rows& rows, OracleResult& output) {
  for (std::size_t i = 0; i < output.rows.size(); ++i) {
    auto& history = output.rows[i].history;
    history.row = type25_friction_test::EndOracle(history.row); // Original MAINF reset precedes KEEPF.
    rows.Store(i,history.row);
  }
  const auto count = output.occurrences.size();
  std::vector<int> candidates_n(count), candidates_e(count);
  std::vector<double> penetration(4*count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto& item = output.occurrences[i]; candidates_n[i] = item.secondary; candidates_e[i] = item.local_main;
    for (unsigned k = 0; k < 4; ++k) penetration[4*i+k] = item.cache.sector[k].penetration;
  }
  const int counts[]{Integer(input.source.secondary_count),Integer(input.source.main_count),
      Integer(count),1,Integer(table.node_ids.size())};
  // Run the complete original routine over one next INDEX at a time. This is
  // exactly its source loop order and preserves earlier marker clears. It also
  // lets the adapter reject a masked cache only when the native next branch
  // would actually consume it, without fabricating unused barycentrics/PENM.
  for (std::size_t i = 0; i < count; ++i) {
    if (candidates_n[i] <= 0) continue;
    const auto row = std::size_t(candidates_n[i]-1); const int before = rows.markers[4*row];
    Require(before != std::numeric_limits<int>::min(), "Native KEEPF marker abs would overflow");
    if (std::abs(before) == table.main_global[std::size_t(candidates_e[i]-1)]) {
      for (const auto& cache : output.occurrences[i].cache.sector) {
        Require((cache.defined & s::PenetrationDefined) != 0, "Native KEEPF would read an undefined cache sum");
        Finite(cache.penetration);
      }
    }
    const int index = Integer(i+1);
    rd_lifecycle_keep(counts,candidates_n.data(),candidates_e.data(),&index,table.main_global.data(),
        rows.markers.data(),penetration.data(),rows.penetration.data(),table.node_ids.data(),
        table.secondary_nodes.data(),rows.friction.data(),rows.metrics.data(),rows.stiffness.data());
    if (before != 0 && rows.markers[4*row] == 0) ++output.rows[row].zero_sum_resets;
    output.occurrences[i].secondary = candidates_n[i];
    if (candidates_n[i] > 0) ++output.rows[row].kept_count;
  }
  for (std::size_t i = 0; i < output.rows.size(); ++i) rows.Load(i,output.rows[i].history.row);
}
} // namespace type25_lifecycle_test::reference
