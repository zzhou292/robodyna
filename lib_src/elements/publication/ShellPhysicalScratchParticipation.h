// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../../solvers/FENodalState.h"
#include <cstddef>
#include <cstdint>

namespace tlfea::contact {
class SelfContactTransaction;
}

namespace tl::fea {

class ShellBatchPublication;
struct ShellPublicationReport;

// Closed contact scratch roster.  Values are ordered deliberately so all
// publication checks visit the fixed wall slot before the self-contact slot.
enum class ShellPhysicalScratchContributorKind : std::uint8_t {
  MappedWall = 0,
  SelfContact = 1,
};

class ShellPhysicalScratchParticipation;

// Copyable authority for one contributor's completed accepted-force and
// candidate-validation phases.  A default value is invalid.  All authority
// fields and the constructor for a valid value are private, so a descriptive
// NodalValidationReceipt or caller-authored field aggregate cannot substitute.
class ShellPhysicalScratchParticipationReceipt {
 public:
  ShellPhysicalScratchParticipationReceipt() noexcept = default;
  ShellPhysicalScratchContributorKind kind() const noexcept { return kind_; }
  std::uint64_t source_id() const noexcept { return source_id_; }
  std::uint64_t generation() const noexcept { return generation_; }
  bool valid() const noexcept {
    return issuer_ != nullptr && publication_ != nullptr && owner_ != nullptr &&
        source_id_ != 0 && generation_ != 0 && prepared_.attempt != 0;
  }

 private:
  friend class ShellBatchPublication;
  friend class ShellPhysicalScratchParticipation;
  const ShellPhysicalScratchParticipation* issuer_ = nullptr;
  const ShellBatchPublication* publication_ = nullptr;
  const FENodalState* owner_ = nullptr;
  ShellPhysicalScratchContributorKind kind_ =
      ShellPhysicalScratchContributorKind::MappedWall;
  std::uint64_t source_id_ = 0, generation_ = 0;
  NodalPreparedView prepared_;
};

// A concrete mapped-wall or self-contact module owns one of these fixed-size
// issuers privately.  It owns only host attempt identity/generation scratch:
// no force, mechanical state, accepted contact history, allocation, stream,
// owner commit, or device operation.  Calls are serialized with the owner.
class ShellPhysicalScratchParticipation {
 public:
  ShellPhysicalScratchParticipation() noexcept = default;
  ~ShellPhysicalScratchParticipation() noexcept;
  ShellPhysicalScratchParticipation(
      const ShellPhysicalScratchParticipation&) = delete;
  ShellPhysicalScratchParticipation& operator=(
      const ShellPhysicalScratchParticipation&) = delete;
  ShellPhysicalScratchParticipation(
      ShellPhysicalScratchParticipation&&) = delete;
  ShellPhysicalScratchParticipation& operator=(
      ShellPhysicalScratchParticipation&&) = delete;

  // Mapped-wall public path only. SelfContact always rejects here; its
  // transaction is the sole friend allowed to record accepted force/STI.
  ShellPublicationReport RecordAcceptedAssembly(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&) noexcept;
  // Mapped-wall public path only. SelfContact always rejects here; its
  // transaction seals only after complete candidate/activity/interval checks.
  // Output is unchanged on failure.
  ShellPublicationReport SealCandidate(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalPreparedView&,
      ShellPhysicalScratchParticipationReceipt*) noexcept;
  // Revokes only this issuer's current attempt scratch.  The common publisher
  // also invokes this from every configured transaction discard.
  void DiscardTrial() noexcept;

  bool configured() const noexcept { return publication_ != nullptr; }
  ShellPhysicalScratchContributorKind kind() const noexcept { return kind_; }
  std::uint64_t source_id() const noexcept { return source_id_; }
  std::uint64_t generation() const noexcept { return generation_; }

 private:
  friend class ShellBatchPublication;
  friend class ::tlfea::contact::SelfContactTransaction;
  enum class Phase : std::uint8_t { Idle, AssemblyRecorded, CandidateSealed };
  ShellPublicationReport RecordSelfContactAcceptedAssembly(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&) noexcept;
  ShellPublicationReport SealSelfContactCandidate(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalPreparedView&,
      ShellPhysicalScratchParticipationReceipt*) noexcept;
  void Bind(ShellBatchPublication&, FENodalState&,
      ShellPhysicalScratchContributorKind, std::uint64_t,
      std::size_t witness_count) noexcept;
  void Unbind(const ShellBatchPublication*) noexcept;
  void Consume() noexcept;

  ShellBatchPublication* publication_ = nullptr;
  FENodalState* owner_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::uint64_t source_id_ = 0, owner_id_ = 0, base_epoch_ = 0;
  std::uint64_t attempt_ = 0, last_base_epoch_ = 0, last_attempt_ = 0;
  std::uint64_t generation_ = 0;
  std::size_t witness_count_ = 0;
  ShellPhysicalScratchContributorKind kind_ =
      ShellPhysicalScratchContributorKind::MappedWall;
  Phase phase_ = Phase::Idle;
};

struct ShellPhysicalScratchRosterEntry {
  ShellPhysicalScratchParticipation* issuer = nullptr;
  std::uint64_t source_id = 0;
};
struct ShellPhysicalScratchRoster {
  ShellPhysicalScratchRosterEntry mapped_wall;
  ShellPhysicalScratchRosterEntry self_contact;
};
struct ShellPhysicalScratchReceiptRoster {
  const ShellPhysicalScratchParticipationReceipt* mapped_wall = nullptr;
  const ShellPhysicalScratchParticipationReceipt* self_contact = nullptr;
};

inline constexpr std::size_t MaxShellPhysicalScratchParticipationHostBytes =
    4096;
struct ShellPhysicalScratchParticipationLimits {
  std::size_t max_host_bytes =
      MaxShellPhysicalScratchParticipationHostBytes;
};
struct ShellPhysicalScratchParticipationForecast {
  // Publication-owned fixed roster/seal state is allocated once at
  // configuration.  Issuers are caller-owned fixed objects, normally embedded
  // in the concrete contact modules.
  std::size_t publication_host_bytes = 0;
  std::size_t configured_issuer_host_bytes = 0;
  std::size_t total_host_bytes = 0;
};

}  // namespace tl::fea
