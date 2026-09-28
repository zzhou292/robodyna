// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../ShellBatchPublication.h"
#include <cstddef>
#include <cstdint>
#include <memory>
namespace tl::fea {
class PhysicalActivitySnapshot;
namespace physical_activity { struct State; struct BatchAccess; }
enum class PhysicalActivityStatus : std::uint8_t {
  Ok, InvalidInput, NotInitialized, AlreadyInitialized, ResourceLimit,
  SourceMismatch, OwnerFailure, PublicationFailure, FamilyFailure,
  InvalidActivity, Reactivation, UnsupportedRemoval, StaleReceipt, DeviceFailure
};
enum class PhysicalActivityFamily : std::uint8_t {
  None, Qeph, T3, Qbat, Type25, Type13, Solids, Beam18, Type45
};
enum class PhysicalActivityStage : std::uint8_t {
  None, Point, Mixed, Failure, Force, Agreement, PointIdentity, Transition
};
struct PhysicalActivityReport {
  PhysicalActivityStatus status = PhysicalActivityStatus::Ok;
  const char* message = "OK";
  PhysicalActivityFamily family = PhysicalActivityFamily::None;
  PhysicalActivityStage stage = PhysicalActivityStage::None;
  std::size_t family_index = SIZE_MAX;
  std::uint32_t detail = 0;
  NodalStatus owner_status = NodalStatus::Ok;
  ShellPublicationStatus publication_status = ShellPublicationStatus::Success;
};
struct PhysicalActivityLimits {
  std::size_t max_family_parents = 524288;
  std::size_t max_host_bytes = 1u << 20;
  std::size_t max_startup_host_bytes = 2u << 20;
  std::size_t max_device_bytes = 8u << 20;
};
struct PhysicalActivityForecast {
  std::size_t qeph_count = 0, t3_count = 0, qbat_count = 0;
  std::size_t owned_host_bytes = 0, startup_host_bytes = 0, preparation_host_bytes = 0;
  std::size_t device_bytes = 0, preparation_device_bytes = 0;
};
struct PhysicalActivityFamilySummary {
  std::size_t count = 0, active_count = 0, first_inactive = SIZE_MAX;
  std::size_t removed_count = 0, first_removed = SIZE_MAX;
};
struct PhysicalActivityFamilyView {
  const std::uint8_t* base = nullptr;
  const std::uint8_t* current = nullptr;
  PhysicalActivityFamilySummary summary;
};
// Borrowed for one uninterrupted authenticated consumer operation on stream.
// QEPH/T3 use their complete ShellBatchBinding family indices. Other physical
// contributors are admitted only under the preserved all-active obligations;
// this view neither deletes nodes/mass nor decides native contact topology.
struct PhysicalActivityDeviceView {
  PhysicalActivityFamilyView qeph, t3;
  std::size_t qbat_count = 0, type45_count = 0;
  NodalStamp accepted;
  std::uint64_t attempt = 0, generation = 0;
  cudaStream_t stream = nullptr;
};
class PhysicalAcceptedActivityReceipt {
 public:
  PhysicalAcceptedActivityReceipt() noexcept = default;
  bool valid() const noexcept;
 private:
  friend class PhysicalActivitySnapshot;
  friend struct physical_activity::State;
  std::weak_ptr<physical_activity::State> state_;
  std::uint64_t generation_ = 0;
};
class PhysicalPreparedActivityReceipt {
 public:
  PhysicalPreparedActivityReceipt() noexcept = default;
  bool valid() const noexcept;
 private:
  friend class PhysicalActivitySnapshot;
  friend struct physical_activity::State;
  std::weak_ptr<physical_activity::State> state_;
  std::uint64_t generation_ = 0;
};
} // namespace tl::fea
