// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include <vector>
namespace tlfea::contact::radioss_type25::runtime_detail {
// Startup-only staging. Destroyed after all copies drain; no second nodal owner.
struct SourceStaging {
  std::vector<std::uint64_t> ids,removal_offsets;
  std::vector<int> codes;
  std::vector<std::uint32_t> secondary_nodes,main_nodes,removal_nodes;
  std::vector<candidates::Main> primary;
  std::vector<NativeGeometryHistory> history;
  std::vector<Vector> positions;
  std::vector<double> native_mass,secondary_stiffness,secondary_gaps,main_stiffness,main_gaps,main_curvature;
  candidates::Source inventory;
  search::Source maintenance;
  std::size_t bytes=0;
};
TransactionReport PrepareSource(const TransactionConfig&,const FixedMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,SourceStaging&) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail
