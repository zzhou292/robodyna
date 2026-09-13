// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactActiveUseBinding.h"
#include "SelfContactForceTypes.h"
#include "SelfContactForceValues.h"

#include <memory>

namespace tlfea::contact {

// Accepted-state, memoryless CUDA scratch contributor for an explicit bounded
// batch of already discovered/resolved directed VF events. It owns no
// broadphase, crossing query, candidate geometry, history or commit.
class SelfContactForceAssembly {
 public:
  SelfContactForceAssembly() noexcept;
  ~SelfContactForceAssembly();
  SelfContactForceAssembly(const SelfContactForceAssembly&) = delete;
  SelfContactForceAssembly& operator=(const SelfContactForceAssembly&) = delete;

  static SelfContactForcePreflight Forecast(
      const SelfContactForceConfig&,
      const SelfContactActiveUseBinding&,
      SelfContactForceLimits = {}) noexcept;

  SelfContactForceReport Initialize(
      const SelfContactForceConfig&,
      const SelfContactActiveUseBinding&,
      tl::fea::FENodalState&,
      SelfContactForceLimits = {});

  SelfContactForceReport AssembleAccepted(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&,
      SelfContactForceEventView,
      SelfContactForceAssemblyReceipt*);

  void DiscardTrial() noexcept;
  tl::fea::NodalAllocationInfo allocations() const noexcept;
  SelfContactForceForecast forecast() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tlfea::contact
