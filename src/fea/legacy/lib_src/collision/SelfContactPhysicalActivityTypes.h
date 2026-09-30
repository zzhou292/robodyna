// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactActiveUseTypes.h"
#include "lib_src/elements/ShellBatchPublication.h"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace tlfea::contact {

enum class SelfContactPhysicalActivityStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  ResourceLimit,
  IdentityMismatch,
  OwnerFailure,
  PublicationFailure,
  AssemblyIncomplete,
  QephFailure,
  T3Failure,
  QbatFailure,
  InvalidActivity,
  Reactivation,
  StaleReceipt,
};

struct SelfContactPhysicalActivityReport {
  SelfContactPhysicalActivityStatus status =
      SelfContactPhysicalActivityStatus::Ok;
  tl::fea::ShellBindingFamily family =
      tl::fea::ShellBindingFamily::None;
  std::size_t parent = SIZE_MAX;
  std::size_t family_index = SIZE_MAX;
  tl::fea::ShellPublicationStatus publication_status =
      tl::fea::ShellPublicationStatus::Success;
  tl::fea::NodalStatus owner_status = tl::fea::NodalStatus::Ok;
  const char* message = "OK";
};

struct SelfContactPhysicalActivityLimits {
  std::size_t max_selected_parents = 2048;
  std::size_t max_family_parents = 2048;
  std::size_t max_host_bytes = 64u << 20;
  std::size_t max_startup_host_bytes = 64u << 20;
  static constexpr SelfContactPhysicalActivityLimits Vehicle() noexcept {
    return {524288, 524288, 512u << 20, 512u << 20};
  }
};

struct SelfContactPhysicalActivityForecast {
  std::size_t selected_parent_count = 0;
  std::size_t qeph_parent_count = 0;
  std::size_t t3_parent_count = 0;
  std::size_t qbat_parent_count = 0;
  // One retained allocation: accepted/current selected-parent values followed
  // by three exact complete-family readback arrays.
  std::size_t arena_bytes = 0;
  std::size_t owned_host_bytes = 0;
  std::size_t startup_host_bytes = 0;
  std::size_t host_allocations = 0;
};

struct SelfContactPhysicalActivityPreflight {
  SelfContactPhysicalActivityReport report;
  SelfContactPhysicalActivityForecast forecast;
};

struct SelfContactPhysicalActivityAllocationInfo {
  std::size_t host_bytes = 0;
  std::size_t host_allocations = 0;
};

namespace self_contact_physical_activity {
struct State;
}

class SelfContactPhysicalActivity;

class SelfContactAcceptedActivityReceipt {
 public:
  SelfContactAcceptedActivityReceipt() noexcept = default;
  bool valid() const noexcept;
  // Accepted authority is base==current. Empty after prepared capture,
  // discard, authority destruction, owner publication, or generation change.
  SelfContactActivityView activity() const noexcept;

 private:
  friend class SelfContactPhysicalActivity;
  friend struct self_contact_physical_activity::State;
  std::weak_ptr<self_contact_physical_activity::State> state_;
  const void* buffer_identity_ = nullptr;
  std::uint64_t generation_ = 0;
  std::uint64_t owner_id_ = 0;
  std::uint64_t base_epoch_ = 0;
  std::uint64_t attempt_ = 0;
};

class SelfContactPreparedActivityReceipt {
 public:
  SelfContactPreparedActivityReceipt() noexcept = default;
  bool valid() const noexcept;
  // Returns the retained accepted base and actual prepared current arrays only
  // while this exact prepared generation remains live.
  SelfContactActivityView activity() const noexcept;

 private:
  friend class SelfContactPhysicalActivity;
  friend struct self_contact_physical_activity::State;
  std::weak_ptr<self_contact_physical_activity::State> state_;
  const void* base_buffer_identity_ = nullptr;
  const void* current_buffer_identity_ = nullptr;
  std::uint64_t generation_ = 0;
  std::uint64_t owner_id_ = 0;
  std::uint64_t base_epoch_ = 0;
  std::uint64_t attempt_ = 0;
};

}  // namespace tlfea::contact
