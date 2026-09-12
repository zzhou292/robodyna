// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QbatBatchTypes.h"
#include "../../ShellNodalStiffness.h"
#include "../../mapped_shell/NodeGather.h"
#include "../../mapped_shell/ObserverTypes.h"
#include "MeasurementTypes.h"
namespace tl::fea::qbat::mapped {
struct AssemblyParent {
  shell_nodal_stiffness::Packet<4> stiffness;
  BatchStatus status=BatchStatus::Success;
  std::uint32_t node=UINT32_MAX;
};
struct MaximumSummary { double value; bool valid; };
using AssemblyNode=mapped_shell::AssemblyNode;
struct AssemblyMemory {
  std::uint32_t* offsets=nullptr;
  std::uint32_t* incidence=nullptr;
  AssemblyParent* parent=nullptr;
  AssemblyNode* node=nullptr;
  unsigned long long* failure=nullptr;
  MaximumSummary* maximum=nullptr;
  MeasurementParent* measurement=nullptr;
};
struct ForceAccess {
  const BatchResult* accepted;
  // QBAT's admitted four-point rows are constitutive. No mixed-section/skin
  // array exists in this family, so none is fabricated for the gather.
  TL_QBAT_HD bool Skip(std::size_t) const noexcept { return false; }
  TL_QBAT_HD const Vec3* Force(std::size_t p,unsigned slot) const noexcept {
    return accepted[p].internal_force_n+slot;
  }
  TL_QBAT_HD const Vec3* Couple(std::size_t p,unsigned slot) const noexcept {
    return accepted[p].internal_couple_nm+slot;
  }
};
inline unsigned MaximumBlocks(std::size_t nodes) noexcept {
  return mapped_shell::ObserverBlocks(0,nodes);
}
} // namespace tl::fea::qbat::mapped
