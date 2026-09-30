// SPDX-License-Identifier: AGPL-3.0-or-later
// QEPH conventions adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Source pin/map: qualification/qeph/source-manifest.json; see LICENSE.md.
#pragma once
#include "QephData.h"

#ifndef TL_QEPH_HD
#if defined(__CUDACC__)
#define TL_QEPH_HD __host__ __device__
#else
#define TL_QEPH_HD
#endif
#endif

namespace tl::fea::qeph {
struct HistoryValues {
  // Material order XX,YY,XY,YZ,ZX; Q3b rate order instead uses XZ,YZ.
  double stress[5]{};           // FOR, Pa, includes instantaneous DM stress.
  double material_stress[5]{};  // FOR_G, Pa, excludes that instantaneous term.
  double bending_stress[3]{};   // MOM, Pa; physical moment/length=t_eff^2*MOM.
  double stabilization[12]{};   // HOURG; Pa/m at 2,3,8,9; other entries Pa.
  double strain_curvature[8]{}; // STRA, material order then KXX,KYY,KXY.
  double thickness=0;          // Reported THK; ITHK0 force thickness stays fixed.
  double internal_work[2]{};    // Accumulated native EINT, J; not a potential.
  double hourglass_viscous_work=0; // One-cell PARTSAV(8), J.
  double active=1;             // Accepted OFF 0/1; legacy entry points admit only active1.
};
struct HistoryStamp { double time=0; std::uint64_t sample_index=0; };

// Trivially-copyable material value, not an allocation/state owner or clock.
// The reference binding is exact input identity, not authenticated provenance.
class History {
 public:
  TL_QEPH_HD bool prepared() const noexcept { return prepared_; }
  TL_QEPH_HD const HistoryValues& data() const noexcept { return data_; }
  TL_QEPH_HD const HistoryStamp& stamp() const noexcept { return stamp_; }
  TL_QEPH_HD bool matches_reference(const ReferenceData&) const noexcept;
 private:
  TL_QEPH_HD static Status Prepare(const ReferenceData&,const HistoryValues&,HistoryStamp,History&,bool) noexcept;
  friend TL_QEPH_HD Status PrepareFailurePrescribedHistory(const ReferenceData&,const HistoryValues&,HistoryStamp,History&) noexcept;
  HistoryValues data_{};
  HistoryStamp stamp_{};
  ReferenceInput reference_input_{};
  bool prepared_=false;
  friend TL_QEPH_HD Status PreparePrescribedHistory(const ReferenceData&,
      const HistoryValues&,HistoryStamp,History&) noexcept;
};
} // namespace tl::fea::qeph
