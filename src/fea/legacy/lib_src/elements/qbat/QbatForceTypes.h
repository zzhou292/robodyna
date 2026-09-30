// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected native QBAT packets: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatGeometry.h"
#include "lib_src/elements/qeph/QephKinematicsData.h"
#include "lib_src/elements/qeph/QephHistoryData.h"
#include "lib_src/materials/Law44MembranePlasticity.h"
#include "lib_src/materials/failure/ShellConstantPlasticFailure.h"

namespace tl::fea::qbat {
using Material = tl::material::TabulatedShellPlasticityParameters;
using PointHistory = tl::material::TabulatedShellPlasticityHistory;
using PointResult = tl::material::TabulatedShellPlasticityResult;
using Failure = tl::material::failure::ConstantPlasticFailureParameters;
using FailureHistory = tl::material::failure::ConstantPlasticFailureHistory;
using PrescribedInterval = qeph::PrescribedInterval;
using HistoryStamp = qeph::HistoryStamp;

struct SurfaceHistory {
  PointHistory material;
  FailureHistory failure;
  bool surface_active=true; // Native OFFPG, distinct from parent OFF.
  double force_stress_pa[5]{}; // Cached FORPG, including native viscosity.
  double strain[8]{}; // STRPG: XX YY XY YZ ZX KXX KYY KXY.
};
struct HistoryValues {
  SurfaceHistory point[4];
  double force_stress_pa[5]{}; // Native ordered quarter mean of FORPG.
  double strain[8]{}; // GBUF%STRA, accumulated in point order.
  double thickness_m=0;
  double internal_work_j[2]{}; // EINT membrane/bending; not WPLA+EVIS.
  double plastic_work_j=0;
  double numerical_viscous_work_j=0; // Native PARTSAV(8), CBAVISC+CBAVISNP1.
  double reported_rate_per_s=0; // CBAFORC3 global rate, ASRATE=1.
  bool element_active=true;
};
class History {
 public:
  TL_QBAT_HD bool prepared() const noexcept { return prepared_; }
  TL_QBAT_HD const HistoryValues& data() const noexcept { return data_; }
  TL_QBAT_HD HistoryStamp stamp() const noexcept { return stamp_; }
  TL_QBAT_HD const Reference& reference() const noexcept { return reference_; }
  TL_QBAT_HD const Material& material() const noexcept { return material_; }
  TL_QBAT_HD Failure failure() const noexcept { return failure_; }
 private:
  HistoryValues data_;
  HistoryStamp stamp_;
  Reference reference_;
  Material material_;
  Failure failure_;
  bool prepared_=false;
  friend TL_QBAT_HD Status PreparePrescribedHistory(const Reference&,const Material&,
      Failure,const HistoryValues&,HistoryStamp,History&) noexcept;
};
struct Kinematics {
  Geometry geometry;
  Vec3 corrected_velocity[3]{}; // Packed V13/V24/VHI, m/s.
  double local_spin[8]{}; // Native RXYZ: Rx13 Ry13 Rx24 Ry24 RxHI RyHI RxSUM RySUM.
  double rate[4][8]{}; // VDEF: XX YY XY XZ YZ KXX KYY KXY.
  double strain_increment[4][8]{}; // Material order: XX YY XY YZ ZX KXX KYY KXY.
  double equivalent_rate_per_s[4]{};
  double characteristic_length_m=0;
  double nodal_factor[2]{}; // FACN, before stiffness scatter.
};
struct PointObservation {
  PointResult material; // Current unmasked stress; saved stress is separate.
  double thickness_before_m=0,thickness_material_m=0,thickness_after_m=0;
  double force_volume_m3=0;
  bool failed_now=false;
};
struct ForceDiagnostics {
  double membrane_viscosity=0,numerical_viscosity=0;
  double sound_speed_m_s=0,viscosity_timestep_factor=0;
  double unscaled_element_dt_s=0,translation_stiffness_n_m=0;
  double rotation_stiffness_nm=0; // Selected NODADT0 CNDT3 is exactly zero.
  double internal_work_increment_j[2]{},plastic_work_increment_j=0;
  double numerical_viscous_work_increment_j=0;
  bool removed_now=false;
};
struct ForceTrial {
  History proposed_history;
  Kinematics kinematics;
  PointObservation point[4];
  Vec3 internal_force_n[4]{}; // Positive native internal force; RHS subtracts it.
  Vec3 internal_couple_nm[4]{}; // NPTT1/IDRIL0 native VM stays zero.
  ForceDiagnostics diagnostics;
};
} // namespace tl::fea::qbat
