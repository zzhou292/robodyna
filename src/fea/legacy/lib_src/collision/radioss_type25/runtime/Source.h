// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../current_normals/Types.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include <vector>
namespace tlfea::contact::radioss_type25::runtime_detail {
struct MovingSourceStaging {
  startup::Snapshot starter;
  current_normals::Topology topology;
  normal_activation::Profile activation;
  std::vector<std::uint32_t> free_main_ids;
  std::vector<double> main_coefficients;
  bool enabled=false,mixed=false;
};
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
  MovingSourceStaging moving;
};
TransactionReport PrepareSource(const TransactionConfig&,const FixedMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,SourceStaging&) noexcept;
TransactionReport PrepareSource(const TransactionConfig&,const MovingMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,SourceStaging&) noexcept;
TransactionReport PrepareSource(const TransactionConfig&,const MixedMovingMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,SourceStaging&) noexcept;
// Shared physical and scalar admission. Only the explicit source overloads
// select require_fixed; no caller-selected flag expands the public profile.
TransactionReport PrepareSourceChecked(const TransactionConfig&,const ContactSourceInput&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,bool require_fixed,SourceStaging&,const startup::Snapshot* mixed=nullptr) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail
