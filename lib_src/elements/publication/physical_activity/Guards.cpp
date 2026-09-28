// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tl::fea::physical_activity {
PhysicalActivityReport GuardOtherFamilies(const ShellPhysicalBinding& source,
    const ShellPhysicalDiagnostics& d) noexcept {
  using S = PhysicalActivityStatus; using F = PhysicalActivityFamily;
  const auto check = [](std::size_t expected, std::size_t count, std::size_t active, F family) {
    PhysicalActivityReport report;
    report.family = family;
    if (count != expected) {
      report.status = S::SourceMismatch; report.message = "Physical activity source count differs";
    } else if (active != count) {
      report.status = S::UnsupportedRemoval;
      report.message = "This physical family has no admitted contact removal lifecycle";
    }
    return report;
  };
  const auto& s = source.coefficients()->scope();
  auto r = d.has_qbat ? check(source.shells()->qbat_count(), d.qbat.element_count, d.qbat.active_count, F::Qbat) : PhysicalActivityReport{};
  if (r.status == S::Ok && d.has_type25) r = check(s.type25_connections, d.type25.element_count, d.type25.active_count, F::Type25);
  if (r.status == S::Ok && d.has_type13) r = check(s.type13_connections, d.type13.element_count, d.type13.active_count, F::Type13);
  if (r.status != S::Ok) return r;
  if (d.has_solids) {
    const std::size_t expected[]{s.solid18_parents, s.solid24_parents, s.solid6z_parents,
        s.solid18_law44_parents, s.solid18_law90_parents};
    for (unsigned k = 0; k < 5; ++k) if (d.solids.parent_count[k] != expected[k])
      return {S::SourceMismatch, "Solid activity scope differs", F::Solids, PhysicalActivityStage::None, k};
  }
  if (d.has_beam18 && d.beam18.parent_count != s.beam18_parents)
    return {S::SourceMismatch, "Beam activity scope differs", F::Beam18};
  // Current solid, beam18 and TYPE45 operators reject unsupported removal
  // before complete typed diagnostics exist. Their exact roster and candidate
  // are authenticated by publication, never inferred from missing mask data.
  return {};
}
} // namespace tl::fea::physical_activity
