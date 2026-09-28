// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include <memory>
namespace tlfea::contact::radioss_type25::activity_operands {
// Numerical device staging only. The composing transaction authenticates its
// Plan against the same source and physical participants, and freshly borrows
// the physical activity view before every Stage call. Raw views cannot grant
// physical or contact publication authority. No owner, clock, accepted selector,
// content-generation counter, forces or constitutive histories live here.
class State {
 public:
  State() noexcept;
  ~State();
  State(const State&) = delete;
  State& operator=(const State&) = delete;
  // Source and optional normal topology are the already validated host startup
  // views used to upload the transaction's slot 0. Plan/source backing need not
  // outlive Initialize. The bounded query may inspect CUDA scan requirements
  // but does not allocate or launch kernels.
  // Descriptor-only Preflight admits null device pointers with complete capacities.
  // Initialize additionally requires actual disjoint device ranges.
  static Forecast Preflight(const activity_source::Plan&, const ContactSourceInput&,
      const current_normals::Topology*, UnitScale, BorrowedSlot, Limits = {}) noexcept;
  TransactionReport Initialize(const activity_source::Plan&, const ContactSourceInput&,
      const current_normals::Topology*, UnitScale, BorrowedSlot, cudaStream_t,
      Limits = {}) noexcept;
  // Stage reads only the explicitly selected accepted slot and writes its
  // alternate. Failure leaves accepted bytes untouched and hides staged output.
  // The transaction increments source content generation only when changed,
  // and applies secondary_coefficients to candidate history WITHOUT resetting
  // its existing initial_contact_flag. No publication is performed here.
  StageReport Stage(unsigned accepted_slot,
      const tl::fea::PhysicalActivityDeviceView&) noexcept;
  View view(unsigned slot) const noexcept;
  void DiscardStaged(unsigned accepted_slot, unsigned alternate_slot) noexcept;
  Forecast allocations() const noexcept;
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact::radioss_type25::activity_operands
