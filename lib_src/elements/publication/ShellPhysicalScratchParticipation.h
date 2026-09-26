// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../../solvers/FENodalState.h"
#include <cstddef>
#include <cstdint>

namespace tlfea::contact {
class NodalWallMappedContact;
class SelfContactTransaction;
namespace radioss_type25 {class Transaction;}
}

namespace tl::fea {

class ShellBatchPublication;
class NativeContactPublicationState;
namespace native_contact_publication {class QualificationAccess;}
struct ShellPublicationReport;

// Closed contact scratch kinds. Legacy slots visit mapped wall before self;
// explicit native groups use separately bound ordinal slots, not enum indices.
enum class ShellPhysicalScratchContributorKind : std::uint8_t {
  MappedWall = 0,
  SelfContact = 1,
  NativeContact = 2, // Explicit native-group member; kind is not its slot index.
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
        source_id_ != 0 && issuer_lifetime_id_ != 0 && binding_id_ != 0 &&
        generation_ != 0 && prepared_.attempt != 0;
  }

 private:
  friend class ShellBatchPublication;
  friend class ShellPhysicalScratchParticipation;
  const ShellPhysicalScratchParticipation* issuer_ = nullptr;
  const ShellBatchPublication* publication_ = nullptr;
  const FENodalState* owner_ = nullptr;
  ShellPhysicalScratchContributorKind kind_ =
      ShellPhysicalScratchContributorKind::MappedWall;
  std::uint64_t source_id_ = 0, issuer_lifetime_id_ = 0;
  std::uint64_t binding_id_ = 0, generation_ = 0;
  NodalPreparedView prepared_;
};

// A concrete mapped-wall or self-contact module owns one of these fixed-size
// issuers privately.  It owns only host attempt identity/generation scratch:
// no force, mechanical state, accepted contact history, allocation, stream,
// owner commit, or device operation.  Calls are serialized with the owner.
class ShellPhysicalScratchParticipation {
 public:
  ShellPhysicalScratchParticipation() noexcept;
  ~ShellPhysicalScratchParticipation() noexcept;
  ShellPhysicalScratchParticipation(
      const ShellPhysicalScratchParticipation&) = delete;
  ShellPhysicalScratchParticipation& operator=(
      const ShellPhysicalScratchParticipation&) = delete;
  ShellPhysicalScratchParticipation(
      ShellPhysicalScratchParticipation&&) = delete;
  ShellPhysicalScratchParticipation& operator=(
      ShellPhysicalScratchParticipation&&) = delete;

  // Deliberately closed public probes.  Neither roster kind can self-attest
  // completion through an exposed issuer; concrete transactions are friends.
  ShellPublicationReport RecordAcceptedAssembly(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&) noexcept;
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
  friend class ::tlfea::contact::NodalWallMappedContact;
  friend class ::tlfea::contact::SelfContactTransaction;
  friend class ::tlfea::contact::radioss_type25::Transaction;
  friend class NativeContactPublicationState;
  friend class native_contact_publication::QualificationAccess;
  enum class Phase : std::uint8_t { Idle, AssemblyRecorded, CandidateSealed };
  ShellPublicationReport RecordMappedWallAcceptedAssembly(
      FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&) noexcept;
  ShellPublicationReport SealMappedWallCandidate(
      FENodalState&, const NodalTrialToken&, const NodalPreparedView&,
      ShellPhysicalScratchParticipationReceipt*) noexcept;
  ShellPublicationReport RecordSelfContactAcceptedAssembly(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&) noexcept;
  ShellPublicationReport SealSelfContactCandidate(
      std::uint64_t source_id, FENodalState&, const NodalTrialToken&,
      const NodalPreparedView&,
      ShellPhysicalScratchParticipationReceipt*) noexcept;
  ShellPublicationReport CheckNativeContactAssembly(
      FENodalState&, const NodalTrialToken&, const NodalAssemblyView&) noexcept;
  ShellPublicationReport RecordNativeContactAcceptedAssembly(
      std::uint64_t, FENodalState&, const NodalTrialToken&, const NodalAssemblyView&) noexcept;
  ShellPublicationReport SealNativeContactCandidate(
      std::uint64_t, FENodalState&, const NodalTrialToken&, const NodalPreparedView&,
      ShellPhysicalScratchParticipationReceipt*) noexcept;
  void Bind(ShellBatchPublication&, FENodalState&,
      ShellPhysicalScratchContributorKind, std::uint64_t,
      std::size_t witness_count, std::size_t slot) noexcept;
  void Unbind(const ShellBatchPublication*) noexcept;
  void Consume() noexcept;
  void DetachNativeContactState(NativeContactPublicationState&) noexcept;

  NativeContactPublicationState* native_contact_ = nullptr;
  ShellBatchPublication* publication_ = nullptr;
  FENodalState* owner_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::uint64_t source_id_ = 0, owner_id_ = 0, base_epoch_ = 0;
  std::uint64_t attempt_ = 0, last_base_epoch_ = 0, last_attempt_ = 0;
  std::uint64_t lifetime_id_ = 0, binding_id_ = 0, generation_ = 0;
  std::size_t witness_count_ = 0, slot_ = SIZE_MAX;
  ShellPhysicalScratchContributorKind kind_ =
      ShellPhysicalScratchContributorKind::MappedWall;
  Phase phase_ = Phase::Idle;
};

struct ShellPhysicalScratchRosterEntry {
  ShellPhysicalScratchParticipation* issuer = nullptr;
  std::uint64_t source_id = 0;
};
// First bounded native group capacity. This is a resource bound, not a
// wall/car dispatch. Entry order is the caller-declared interface order.
inline constexpr std::size_t MaxNativeContactInterfaces = 2;
class NativeContactRosterEntry {
 public:
  NativeContactRosterEntry() noexcept = default;
  ShellPhysicalScratchParticipation* issuer() const noexcept { return issuer_; }
  std::uint64_t source_id() const noexcept { return source_id_; }
 private:
  friend class ::tlfea::contact::radioss_type25::Transaction;
  NativeContactRosterEntry(ShellPhysicalScratchParticipation* issuer, std::uint64_t source) noexcept
      : issuer_(issuer), source_id_(source) {}
  ShellPhysicalScratchParticipation* issuer_ = nullptr;
  std::uint64_t source_id_ = 0;
};
struct NativeContactRosterView {
  const NativeContactRosterEntry* entries = nullptr;
  std::size_t count = 0;
};
struct NativeContactReceiptView {
  const ShellPhysicalScratchParticipationReceipt* const* entries = nullptr;
  std::size_t count = 0;
};
struct ShellPhysicalScratchRoster {
  ShellPhysicalScratchRosterEntry mapped_wall;
  ShellPhysicalScratchRosterEntry self_contact;
  // Native group mode is exclusive with both legacy fields. Borrowed for this
  // call only; publisher retains its own bounded normalized entries.
  NativeContactRosterView native_interfaces;
};
struct ShellPhysicalScratchReceiptRoster {
  const ShellPhysicalScratchParticipationReceipt* mapped_wall = nullptr;
  const ShellPhysicalScratchParticipationReceipt* self_contact = nullptr;
  NativeContactReceiptView native_interfaces;
};

inline constexpr std::size_t MaxShellPhysicalScratchParticipationHostBytes =
    4096;
struct ShellPhysicalScratchParticipationLimits {
  std::size_t max_host_bytes =
      MaxShellPhysicalScratchParticipationHostBytes;
  std::size_t max_native_interfaces = MaxNativeContactInterfaces;
};
struct ShellPhysicalScratchParticipationForecast {
  // Publication-owned fixed roster/seal state is allocated once at
  // configuration.  Issuers are caller-owned fixed objects, normally embedded
  // in the concrete contact modules. A complete contact forecast already
  // charges its embedded issuer, so composition adds publication_host_bytes,
  // not total_host_bytes; total is the standalone roster admission cap.
  std::size_t publication_host_bytes = 0;
  std::size_t configured_issuer_host_bytes = 0;
  std::size_t total_host_bytes = 0;
};

}  // namespace tl::fea
