// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellPhysicalPublication.h"
#include "../../assembly/ShellPhysicalBinding.h"
#include "../../constraints/NodalRigidAssemblyBinding.h"
#include <cstdint>
#include <type_traits>

namespace tl::fea::shell_publication_detail {
struct PhysicalScratchParticipationState;

// Exact two-word replacement for the old CIN count header.  A scratch pointer
// is tagged in its guaranteed-zero alignment bit; CIN attachment counts are
// shifted into the same word.  Accessors inspect only the active tagged
// representation, so a legitimate zero witness count is never a sentinel.
struct PhysicalRuntimeState {
  constexpr PhysicalRuntimeState() noexcept = default;
  constexpr void SetCinCounts(std::size_t attachments,
                              std::size_t witnesses) noexcept {
    tagged = static_cast<std::uintptr_t>(attachments) << 1;
    payload = witnesses;
  }
  constexpr bool HasScratchParticipation() const noexcept {
    return (tagged & ScratchTag) != 0;
  }
  constexpr std::size_t CinAttachmentCount() const noexcept {
    return static_cast<std::size_t>(tagged >> 1);
  }
  constexpr std::size_t CinWitnessCount() const noexcept {
    return payload;
  }
  void SetScratchParticipation(PhysicalScratchParticipationState*) noexcept;
  PhysicalScratchParticipationState* ScratchParticipation() noexcept;
  const PhysicalScratchParticipationState* ScratchParticipation()
      const noexcept;

 private:
  static constexpr std::uintptr_t ScratchTag = 1;
  std::uintptr_t tagged = 0;
  std::size_t payload = 0;
};
static_assert(sizeof(PhysicalRuntimeState)==2*sizeof(std::size_t),
    "Optional scratch roster preserves the old physical CIN count header");
static_assert(sizeof(std::uintptr_t)==sizeof(std::size_t),
    "Physical runtime tagging requires the native size_t pointer ABI");
static_assert(std::is_trivially_copyable_v<PhysicalRuntimeState>,
    "Physical runtime state must retain byte-copy semantics");

struct PhysicalState {
  PhysicalState(const ShellPhysicalBinding& p,const NodalRigidAssemblyBinding& r,const type45::Model* j=nullptr)
      : binding(p),rigid(r),joint_model(j?*j:type45::Model{}) {}
  ~PhysicalState();
  PhysicalState(const PhysicalState&)=delete;
  PhysicalState& operator=(const PhysicalState&)=delete;
  void SetCinCounts(std::size_t,std::size_t) noexcept;
  bool HasScratchParticipation() const noexcept;
  PhysicalScratchParticipationState* ScratchParticipation() noexcept;
  const PhysicalScratchParticipationState* ScratchParticipation() const noexcept;
  void DiscardScratchParticipation() noexcept;
  ShellPhysicalBinding binding;
  NodalRigidAssemblyBinding rigid;
  type45::Model joint_model;
  FENodalState* owner = nullptr;
  type13::Batch* beams = nullptr;
  solids::Batch* solids = nullptr;
  type45::Batch* joints = nullptr;
  beam18::Batch* structural_beams = nullptr;
  ShellPhysicalPublicationIdentity identity;
  ShellPhysicalPublicationForecast forecast;
  NodalStamp accepted_stamp;
  ShellPhysicalDiagnostics accepted,candidate;
  PhysicalRuntimeState runtime;
};
bool SamePhysicalDiagnostics(const ShellPhysicalDiagnostics&,
    const ShellPhysicalDiagnostics&) noexcept;
ShellPhysicalCandidates Candidates(const ShellPhysicalDiagnostics&) noexcept;
} // namespace tl::fea::shell_publication_detail
