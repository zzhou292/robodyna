// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid24Reference.h"
#include "lib_src/materials/law42/Caller.h"

namespace tl::fea::solid24 {
using Material = tl::material::law42::Parameters;
enum class ForceStatus { Success, InvalidInput, ReferenceMismatch, UnsupportedProfile,
                         InvalidGeometry, MaterialFailure, NonfiniteResult };
// This force/history API selects engine JCVT1, ICP1, IINT2, ISMSTR10, no ISCTL,
// no Prony, ALE, thermal, small-strain switch, element deletion or mass scaling.
struct PrescribedInterval {
  Vec3 position_m[8]{}, velocity_m_s[8]{}; // Original source slot order.
  double base_time_s = 0, dt_s = 0;
  std::uint64_t sample_index = 0;
};
struct HistoryValues {
  tl::material::law42::CallerHistory material;
  double physical_hourglass[3][4]{}; // Native FHOUR, distinct from nodal force.
};
struct HistoryStamp { double time_s = 0; std::uint64_t sample_index = 0; };
struct ForceGeometry {
  StartupGeometry current;
  Vec3 local_velocity_m_s[8]{};
  double derivative_per_m[3][4]{};
  double hourglass_projection[4][4]{};
  double jacobian_diagonal_m[3]{};
  double material_displacement_gradient[9]{};
  double engineering_rate_per_s[6]{};
};
struct ForceDiagnostics {
  tl::material::law42::CallerResult material;
  double stabilization_work_j = 0;
  double stabilization_modulus_pa = 0;
  double stabilization_viscosity_kg_m_s = 0;
};
class History;
struct ForceTrial;
TL_BRICK_HD ForceStatus InitializeHistory(const Reference&,const Material&,History&) noexcept;
TL_BRICK_HD ForceStatus InitializeForce(const Reference&,const Material&,Vec3,ForceTrial&) noexcept;
TL_BRICK_HD ForceStatus EvaluateForce(const Reference&,const History&,const PrescribedInterval&,
                                     const Material&,ForceTrial&) noexcept;
class History {
 public:
  TL_BRICK_HD bool initialized() const noexcept { return initialized_; }
  TL_BRICK_HD const Reference& reference() const noexcept { return reference_; }
  TL_BRICK_HD const Material& material() const noexcept { return material_; }
  TL_BRICK_HD const HistoryValues& values() const noexcept { return values_; }
  TL_BRICK_HD const HistoryStamp& stamp() const noexcept { return stamp_; }
 private:
  Reference reference_;
  Material material_;
  HistoryValues values_;
  HistoryStamp stamp_;
  bool initialized_ = false;
  friend TL_BRICK_HD ForceStatus InitializeHistory(const Reference&,const Material&,History&) noexcept;
  friend TL_BRICK_HD ForceStatus InitializeForce(const Reference&,const Material&,Vec3,ForceTrial&) noexcept;
  friend TL_BRICK_HD ForceStatus EvaluateForce(const Reference&,const History&,
      const PrescribedInterval&,const Material&,ForceTrial&) noexcept;
};
struct ForceTrial {
  History proposed_history;
  ForceGeometry geometry;
  ForceDiagnostics diagnostics;
  Vec3 rhs_force_n[8]{};
};
} // namespace tl::fea::solid24
