#pragma once
#include "T3Reference.h"

namespace tl::qualification::t3 {
struct HistoryValues {
  std::array<double,5> stress{};          // FOR: XX,YY,XY,YZ,ZX; Pa, current DM included.
  std::array<double,5> material_stress{}; // FOR_G: Pa, persistent material only.
  std::array<double,3> bending_stress{};  // MOM: Pa; physical moment/length=t_eff^2*MOM.
  std::array<double,8> strain_curvature{};// STRA: five dimensionless, three 1/m.
  double thickness=0;                    // Reported THK, m; effective t stays fixed.
  std::array<double,2> internal_work{};   // EINT, J; native signed work, not a potential.
  double equivalent_strain_rate=0;       // EPSD, 1/s, overwritten by each sample.
  double active=1;                       // Only active1 is admitted. No HOURG/EVIS.
};
struct HistoryStamp { double time=0; std::uint64_t sample_index=0; };
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
  friend Status PreparePrescribedHistory(const Reference&,const HistoryValues&,HistoryStamp,History&) noexcept;
};
// Explicit prescribed values, not a native restart/solver-history admission.
// Exact reference-input binding; finite signed work allowed, THK>=native EM30,
// EPSD>=0 and active1. All failure output bytes remain unchanged.
Status PreparePrescribedHistory(const Reference&,const HistoryValues&,HistoryStamp,History&) noexcept;
Status InitializeHistory(const Reference&,HistoryStamp,History&) noexcept;
namespace detail {
inline constexpr unsigned kHistoryValues=26;
bool ValidHistoryValues(const HistoryValues&) noexcept;
void PackHistory(const HistoryValues&,std::array<double,kHistoryValues>&) noexcept;
HistoryValues UnpackHistory(const double*) noexcept;
}
} // namespace tl::qualification::t3
