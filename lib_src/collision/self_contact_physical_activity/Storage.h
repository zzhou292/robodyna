// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactPhysicalActivity.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_physical_activity {

struct Layout {
  tl::util::ArenaRegion accepted;
  tl::util::ArenaRegion current;
  tl::util::ArenaRegion qeph;
  tl::util::ArenaRegion t3;
  tl::util::ArenaRegion qbat;
  std::size_t bytes = 0;
};

struct Buffers {
  std::uint8_t* accepted = nullptr;
  std::uint8_t* current = nullptr;
  std::uint8_t* qeph = nullptr;
  std::uint8_t* t3 = nullptr;
  std::uint8_t* qbat = nullptr;
};

bool MakeLayout(std::size_t selected, std::size_t qeph,
                std::size_t t3, std::size_t qbat,
                std::size_t maximum_bytes, Layout&) noexcept;
Buffers Bind(void*, const Layout&) noexcept;

// Value-only transition proof used by the runtime after all complete-family
// readbacks have succeeded. It accepts 1->0 and 0->0, and reports the first
// malformed/reactivated selected parent without modifying caller storage.
SelfContactPhysicalActivityReport ValidateTransition(
    const std::uint8_t* base, const std::uint8_t* current,
    std::size_t count) noexcept;

enum class Phase : std::uint8_t {
  Idle,
  Accepted,
  Prepared,
  Exhausted,
};

struct State {
  explicit State(const SelfContactActiveUseBinding& selected,
                 const tl::fea::ShellPhysicalBinding& source) noexcept
      : active_use(selected), physical(source) {}

  SelfContactActiveUseBinding active_use;
  tl::fea::ShellPhysicalBinding physical;
  tl::fea::FENodalState* owner = nullptr;
  tl::fea::ShellBatchPublication* publication = nullptr;
  tl::fea::ShellPhysicalParticipants participants;
  tl::fea::ShellPhysicalPublicationIdentity identity;
  SelfContactPhysicalActivityForecast storage_forecast;
  Layout layout;
  tl::util::HostArena arena;
  Buffers buffers;
  std::uint64_t generation = 0;
  std::uint64_t owner_id = 0;
  std::uint64_t base_epoch = 0;
  std::uint64_t attempt = 0;
  Phase phase = Phase::Idle;

  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  bool AdvanceGeneration() noexcept;
  void Invalidate() noexcept;
  bool CurrentOwnerEndpoint() const noexcept;
  bool Authenticates(
      const SelfContactAcceptedActivityReceipt&) const noexcept;
  bool Authenticates(
      const SelfContactPreparedActivityReceipt&) const noexcept;
  SelfContactPhysicalActivityReport RevalidateSources() const noexcept;
};

}  // namespace tlfea::contact::self_contact_physical_activity
