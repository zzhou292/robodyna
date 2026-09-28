// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../../../solvers/NodalTrialIdentity.h"
namespace tl::fea::shell_batch_plasticity_detail {
struct MixedDeviceStorage; struct FailureDeviceStorage; struct OnePointDeviceStorage;
}
namespace tl::fea::physical_activity {
template<class Element, class Force> struct FamilyInput {
  const Element* elements = nullptr;
  const Force* results = nullptr;
  const shell_batch_plasticity_detail::MixedDeviceStorage* mixed = nullptr;
  const shell_batch_plasticity_detail::FailureDeviceStorage* failure = nullptr;
  const shell_batch_plasticity_detail::OnePointDeviceStorage* point = nullptr;
  std::size_t count = 0;
  unsigned slab = 0;
  double time = 0, force_time = 0;
  std::uint64_t epoch = 0, force_epoch = 0;
  bool execution = false;
};
using QephInput = FamilyInput<qeph::QephBatchElement, qeph::ForceTrial>;
using T3Input = FamilyInput<t3::T3BatchElement, t3::ForceTrial>;
// Internal batch friend. Raw descriptors cannot authorize public capture.
struct BatchAccess {
  // Internal complete-count proof. check_source additionally authenticates a
  // caller-supplied source handle, including explicit absence.
  static PhysicalActivityReport Type45(const type45::Batch*, const type45::Model*, bool check_source,
      FENodalState&, ShellBatchPublication&, const ShellPhysicalBinding&,
      bool has_diagnostics, const type45::BatchDiagnostics&) noexcept;
  static PhysicalActivityReport Borrow(qeph::QephBatch&, FENodalState&,
      ShellBatchPublication&, const ShellPhysicalBinding&, const NodalTrialToken&,
      const NodalAssemblyView*, const NodalPreparedView*, const qeph::BatchDiagnostics&, QephInput&);
  static PhysicalActivityReport Borrow(t3::T3Batch&, FENodalState&,
      ShellBatchPublication&, const ShellPhysicalBinding&, const NodalTrialToken&,
      const NodalAssemblyView*, const NodalPreparedView*, const t3::BatchDiagnostics&, T3Input&);
};
} // namespace tl::fea::physical_activity
