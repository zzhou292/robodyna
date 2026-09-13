// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "ShellPhysicalScratchParticipation.h"

namespace tl::fea::shell_publication_detail {

inline constexpr std::size_t PhysicalScratchKindCount = 2;
inline constexpr std::size_t PhysicalScratchKindIndex(
    ShellPhysicalScratchContributorKind kind) noexcept {
  return static_cast<std::size_t>(kind);
}

struct PhysicalScratchParticipationEntry {
  ShellPhysicalScratchParticipation* issuer = nullptr;
  std::uint64_t source_id = 0;
};

// One startup allocation with a fixed two-kind extent.  It retains no contact
// mechanics or accepted history; completion is only for the current owner
// attempt and is revoked by common discard.
struct PhysicalScratchParticipationState {
  std::size_t cin_attachment_count = 0, cin_witness_count = 0;
  PhysicalScratchParticipationEntry entries[PhysicalScratchKindCount];
  const FENodalState* sealed_owner = nullptr;
  cudaStream_t sealed_stream = nullptr;
  std::uint64_t sealed_owner_id = 0, sealed_base_epoch = 0;
  std::uint64_t sealed_attempt = 0;
  std::uint64_t sealed_generation[PhysicalScratchKindCount]{};
  bool sealed = false;
};

}  // namespace tl::fea::shell_publication_detail
