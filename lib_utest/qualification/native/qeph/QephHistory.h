#pragma once

#include "QephReferenceTypes.h"

namespace tl::qualification::qeph {

struct HistoryValues {
  // Native stress order: XX, YY, XY, YZ, ZX, unlike Q1's XZ/YZ rate order.
  std::array<double,5> stress{};           // FOR, Pa, includes instantaneous DM.
  std::array<double,5> material_stress{};  // FOR_G, Pa, excludes that DM addition.
  std::array<double,3> bending_stress{};   // MOM, Pa; physical moment=t_eff^2*MOM.
  // HOURG: membrane1/2,bending1/2,transverse1/2, then their six stiffness states.
  // Units Pa except entries2,3,8,9 (zero-based), which are Pa/m.
  std::array<double,12> stabilization{};
  std::array<double,8> strain_curvature{}; // STRA: XX,YY,XY,YZ,ZX,KXX,KYY,KXY.
  double thickness=0;                    // Reported THK; effective t stays fixed.
  std::array<double,2> internal_work{};   // EINT, J: membrane/transverse,bending.
  double hourglass_viscous_work=0;        // Accumulated one-element PARTSAV(8), J.
  double active=1;
};

struct HistoryStamp {
  double time=0;
  std::uint64_t sample_index=0;
};

class History {
 public:
  bool prepared() const noexcept { return prepared_; }
  const HistoryValues& data() const noexcept { return data_; }
  const HistoryStamp& stamp() const noexcept { return stamp_; }
  bool matches_reference(const Reference&) const noexcept;

 private:
  HistoryValues data_{};
  HistoryStamp stamp_{};
  ReferenceInput reference_input_{};
  bool prepared_=false;
  friend Status PreparePrescribedHistory(const Reference&,const HistoryValues&,
                                         HistoryStamp,History&) noexcept;
};

// Staged, allocation-free value preparation. A supplied finite history is an
// explicit prescribed reference input, not a solver restart/material admission.
Status PreparePrescribedHistory(const Reference&,const HistoryValues&,
                                HistoryStamp,History&) noexcept;
Status InitializeHistory(const Reference&,HistoryStamp,History&) noexcept;

}  // namespace tl::qualification::qeph
