// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PhysicalState.h"
#include "PhysicalChecks.h"
#include "../qeph/QephBatchStorage.h"
#include "../t3/T3BatchStorage.h"
#include "../qbat/QbatBatchStorage.h"
#include "../type25/Type25BatchStorage.h"
#include "../type13/resident/Storage.h"
#include "../solids/resident/Storage.h"

namespace tl::fea::shell_publication_detail {
bool CompletePhysicalParticipants(const ShellPhysicalBinding& binding,
    const ShellPhysicalParticipants& p) noexcept {
  if (!binding.prepared() || !binding.shells() || !binding.coefficients()) return false;
  const auto& shell = *binding.shells();
  const auto& ledger = *binding.coefficients();
  return bool(p.qeph) == (shell.qeph_count() != 0) &&
      bool(p.t3) == (shell.t3_count() != 0) &&
      bool(p.qbat) == (shell.qbat_count() != 0) &&
      bool(p.type25) == bool(ledger.type25()) &&
      bool(p.type13) == bool(ledger.type13()) &&
      bool(p.solids) == bool(ledger.solids());
}
ShellPhysicalCandidates Candidates(const ShellPhysicalDiagnostics& d) noexcept {
  return {d.has_qeph ? &d.qeph : nullptr,d.has_t3 ? &d.t3 : nullptr,
      d.has_qbat ? &d.qbat : nullptr,d.has_type25 ? &d.type25 : nullptr,
      d.has_type13 ? &d.type13 : nullptr,d.has_solids ? &d.solids : nullptr};
}
bool SamePhysicalDiagnostics(const ShellPhysicalDiagnostics& a,
    const ShellPhysicalDiagnostics& b) noexcept {
  return trial_identity::SameStamp(a.base_stamp,b.base_stamp) &&
      a.valid == b.valid && a.kinetic_available == b.kinetic_available &&
      a.has_qeph == b.has_qeph && a.has_t3 == b.has_t3 && a.has_qbat == b.has_qbat &&
      a.has_type25 == b.has_type25 && a.has_type13 == b.has_type13 && a.has_solids == b.has_solids &&
      qeph::batch_detail::SameDiagnostics(a.qeph,b.qeph) &&
      t3::batch_detail::SameDiagnostics(a.t3,b.t3) &&
      qbat::batch_detail::SameDiagnostics(a.qbat,b.qbat) &&
      type25::batch_detail::SameDiagnostics(a.type25,b.type25) &&
      type13::batch_detail::SameDiagnostics(a.type13,b.type13) &&
      solids::batch_detail::SameDiagnostics(a.solids,b.solids);
}
} // namespace tl::fea::shell_publication_detail
