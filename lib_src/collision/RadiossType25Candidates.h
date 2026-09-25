// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/candidates/InventoryTypes.h"
#include <memory>
struct CUstream_st;
namespace tlfea::contact::radioss_type25::candidates {
class Inventory;
class InventoryView {
 public:
  const Pair* pairs() const noexcept {return pairs_;}
  const std::uint64_t* secondary_offsets() const noexcept {return offsets_;}
  std::size_t pair_count() const noexcept {return count_;}
  std::size_t secondary_count() const noexcept {return rows_;}
  QueryStamp stamp() const noexcept {return stamp_;}
 private:
  friend class Inventory;
  std::uint64_t owner_=0,sequence_=0;
  const Pair* pairs_=nullptr;const std::uint64_t* offsets_=nullptr;
  std::size_t count_=0,rows_=0;QueryStamp stamp_;
};
// Numerical staging only: no physical Commit, reference reuse receipt or clock.
// Every Stage attempt and Discard invalidate this instance's previous view.
// Two such owners can be held by the common contact coordinator; the accepted
// instance is left untouched while the other stages the next candidate rebuild.
// Stage always drains the explicit stream before returning. A caller serializes
// this owner and treats all borrowed fields as immutable for that duration.
class Inventory {
 public:
  Inventory();~Inventory();
  Inventory(const Inventory&)=delete;Inventory& operator=(const Inventory&)=delete;
  static Status Preflight(const Source&,Limits,Forecast&) noexcept;
  Status Initialize(const Source&,Limits,CUstream_st*) noexcept;
  Status Stage(const Current&) noexcept;
  InventoryView view() const noexcept;
  bool IsCurrent(const InventoryView&) const noexcept;
  void Discard() noexcept;
  Report last_report() const noexcept;
  Forecast allocations() const noexcept;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact::radioss_type25::candidates
