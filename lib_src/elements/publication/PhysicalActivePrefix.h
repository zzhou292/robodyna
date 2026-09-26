// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellBatchPublication.h"
#include <memory>
namespace tl::fea {
enum class ActivePrefixStatus { Ok, InvalidInput, NotInitialized, AlreadyInitialized,
  ResourceLimit, SourceMismatch, InactiveParent, ReadbackFailure };
enum class ActivePrefixFamily { None, Qeph, T3, Qbat, Type25, Type13, Solids, Beam18, Type45 };
struct ActivePrefixReport {
  ActivePrefixStatus status=ActivePrefixStatus::InvalidInput;
  const char* message="Invalid active physical prefix";
  ActivePrefixFamily family=ActivePrefixFamily::None;
  std::size_t parent=SIZE_MAX;
};
struct ActivePrefixLimits { std::size_t max_host_bytes=2u<<20; };
// Incremental owned payload; immutable source backing is shared and budgeted
// by the composition. Existing participant readback storage is not duplicated.
struct ActivePrefixForecast { std::size_t owned_host_bytes=0,activity_capacity=0; };
// Read-only physical activity predicate. It uses genuine complete publication
// diagnostics and existing validated parent activity APIs. It cannot publish,
// issue a nodal/contact receipt or decide native K/gap/constraint source policy.
// Caller discards the complete attempt after any failed candidate observation.
class PhysicalActivePrefix {
 public:
  PhysicalActivePrefix();
  ~PhysicalActivePrefix();
  PhysicalActivePrefix(const PhysicalActivePrefix&)=delete;
  PhysicalActivePrefix& operator=(const PhysicalActivePrefix&)=delete;
  static ActivePrefixReport Preflight(const ShellPhysicalBinding&,ActivePrefixLimits,
      ActivePrefixForecast&) noexcept;
  ActivePrefixReport Initialize(FENodalState&,ShellBatchPublication&,const ShellPhysicalBinding&,
      const ShellPhysicalParticipants&,const ShellPhysicalPublicationIdentity&,ActivePrefixLimits={});
  ActivePrefixReport CheckAccepted(FENodalState&,ShellBatchPublication&);
  ActivePrefixReport CheckPrepared(FENodalState&,ShellBatchPublication&,const NodalTrialToken&,
      const ShellPhysicalDiagnostics&,const NodalPreparedView&);
  ActivePrefixForecast allocations() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea
