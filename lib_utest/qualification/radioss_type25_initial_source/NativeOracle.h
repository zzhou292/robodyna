// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
#include <cstddef>
#include <vector>
#include "../radioss_type25_search_startup/NativeOracle.h"
#include "../radioss_type25_initial_state/NativeOracle.h"
namespace initial_source_test {
// Neutral native-unit packets: node and solid ordinals are one-based native
// indices. The expected path has no production coefficient/geometry call.
struct Solid {
  int source_id=0,part_id=0;
  std::array<int,8> nodes{};
};
struct InventoryInput {
  std::vector<std::array<double,3>> positions;
  std::vector<std::array<int,4>> mains;
  std::vector<int> types,secondary,main_nodes,codes,skews;
  std::vector<double> coefficients,secondary_coefficients,main_gap,secondary_gap;
  std::vector<std::array<double,4>> corner_gaps;
  std::vector<std::array<int,2>> support;
  std::vector<Solid> solids;
  std::vector<std::vector<int>> removed_nodes;
  double multiplier=0,global_gap=0;
};
struct InventoryResult {
  std::vector<std::array<int,2>> pairs;
  std::vector<std::array<double,4>> corner_gaps;
  double engine_margin=0,distance=0,zone=0,maximum_box=0;
  std::uint64_t initialized_voxel_slots=0;
};
// Private COMMON/module arrays are serialized. Complete original BUC,
// INSOL25, TRIVOX1, STO, COR3T/PEN3A and MY_ORDERS execute in this call.
InventoryResult NativeInventory(const InventoryInput&);
// Same complete original removal oracle, admitting authentic G=P+S instead
// of its older ordinary-shell C++ wrapper's G=2P qualification restriction.
type25_search_startup_test::NativeResult NativeGeometry(const InventoryInput&,std::size_t primaries);
void NativePreparedMain(std::size_t mains,std::vector<std::array<int,4>>& rows);
initial_state_test::NativeInitialHistory FullInitialHistory(
    const tlfea::contact::radioss_type25::search_startup::Input&,
    const type25_startup_test::NativeResult&,
    const std::vector<std::array<double,4>>&,const std::vector<std::array<int,2>>&,int sharp=1);
}
