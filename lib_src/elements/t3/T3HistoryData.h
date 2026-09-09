// SPDX-License-Identifier: AGPL-3.0-or-later
// Native T3 conventions, OpenRadioss (C) 2026 Siemens; see LICENSE.md.
#pragma once
#include "T3Data.h"
namespace tl::fea::t3 {
struct HistoryValues {
  double stress[5]{};           // FOR, XX YY XY YZ ZX; Pa, instantaneous DM included.
  double material_stress[5]{};  // FOR_G, persistent material only, Pa.
  double bending_stress[3]{};   // MOM, XX YY XY, Pa; physical moment/length=t_eff^2*MOM.
  double strain_curvature[8]{}; // First5 dimensionless, last3 curvature in 1/m.
  double thickness=0;           // Reported THK, m; force thickness stays reference t.
  double internal_work[2]{};    // Native accumulated EINT, J; not a potential.
  double equivalent_strain_rate=0; // EPSD, 1/s, overwritten each interval.
  double active=1;              // Fixed active1 branch; no HOURG or EVIS.
};
struct HistoryStamp { double time=0; std::uint64_t sample_index=0; };
class History {
 public:
  TL_T3_HD bool prepared() const noexcept { return prepared_; }
  TL_T3_HD const HistoryValues& data() const noexcept { return data_; }
  TL_T3_HD const HistoryStamp& stamp() const noexcept { return stamp_; }
  TL_T3_HD bool matches_reference(const ReferenceData&) const noexcept;
 private:
  HistoryValues data_{};
  HistoryStamp stamp_{};
  ReferenceInput reference_input_{};
  bool prepared_=false;
  friend TL_T3_HD Status PreparePrescribedHistory(const ReferenceData&,const HistoryValues&,
      HistoryStamp,History&) noexcept;
};
} // namespace tl::fea::t3
