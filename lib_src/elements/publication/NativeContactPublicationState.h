// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../solvers/FENodalState.h"
#include <cstdint>
namespace tlfea::contact::radioss_type25 {class Transaction;}
namespace tl::fea {
class ShellBatchPublication;
class ShellPhysicalScratchParticipation;
namespace native_contact_publication {class QualificationAccess;}
// Bounded selectors for startup-owned native history slabs and paired
// maintenance/inventory objects. No mechanics buffers or independent clock.
struct NativeContactSelectors {
  unsigned history=0,reference=0;
  std::uint64_t reference_generation=0;
  bool has_reference=false;
};
// available proves binding/stamp coherence only. Initial generation0 has no
// retained inventory/reference; callers must inspect selectors.has_reference.
struct NativeContactPublicationSnapshot {
  NodalStamp stamp; // Common publication at X_(n+1).
  NodalStamp force_base_stamp; // Contact force/history evaluation at accepted X_n.
  NativeContactSelectors selectors;
  std::uint64_t generation=0;
  bool available=false,force_phase_available=false;
};
// A closed typed participant. Mutation is private to the native transaction and
// the common publisher; there are no user callbacks or public self-attestations.
// All calls/readers serialize with that publisher. Native transaction storage
// has a stable address; neither this participant nor its transaction can move.
class NativeContactPublicationState {
 public:
  NativeContactPublicationState() noexcept=default;
  ~NativeContactPublicationState() noexcept;
  NativeContactPublicationState(const NativeContactPublicationState&)=delete;
  NativeContactPublicationState& operator=(const NativeContactPublicationState&)=delete;
  NativeContactPublicationState(NativeContactPublicationState&&)=delete;
  NativeContactPublicationState& operator=(NativeContactPublicationState&&)=delete;
  NativeContactPublicationSnapshot Accepted(const FENodalState&) const noexcept;
 private:
  friend class ShellBatchPublication;
  friend class ShellPhysicalScratchParticipation;
  friend class ::tlfea::contact::radioss_type25::Transaction;
  friend class native_contact_publication::QualificationAccess;
  // Called only after the concrete native source/owner binding is authenticated.
  bool Attach(FENodalState&,std::uint64_t,ShellPhysicalScratchParticipation&) noexcept;
  bool CanBind(const FENodalState&,std::uint64_t) const noexcept;
  void Bind(ShellBatchPublication&,FENodalState&,std::uint64_t,std::uint64_t) noexcept;
  void Unbind() noexcept;
  // Called after full native force/history/candidate validation, before the
  // issuer seals the same authentic prepared view. Private staging is not commit.
  bool Stage(const NodalPreparedView&,NativeContactSelectors) noexcept;
  bool Ready(const FENodalState&,const NodalPreparedView&,std::uint64_t) const noexcept;
  void Discard() noexcept;
  void Publish(const NodalStamp&) noexcept;
  ShellPhysicalScratchParticipation* issuer_=nullptr;
  ShellBatchPublication* publication_=nullptr;
  FENodalState* owner_=nullptr;
  std::uint64_t source_id_=0,issuer_lifetime_=0,binding_id_=0;
  std::uint64_t generation_=0,staged_issuer_generation_=0;
  NodalStamp accepted_stamp_,force_base_stamp_;
  NodalPreparedView prepared_;
  NativeContactSelectors accepted_,staged_;
  bool attached_=false,pending_=false,force_phase_available_=false;
};
} // namespace tl::fea
