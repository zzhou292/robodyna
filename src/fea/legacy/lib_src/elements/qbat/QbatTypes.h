// SPDX-License-Identifier: AGPL-3.0-or-later
// Native field conventions: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "lib_src/elements/qeph/QephData.h"

#if defined(__CUDACC__)
#define TL_QBAT_HD __host__ __device__
#else
#define TL_QBAT_HD
#endif

namespace tl::fea::qbat {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;
using Status = qeph::Status;

// These shared packets contain only CNEVECI/CDERII/CINMAS values. For IHBE11,
// INER_9_12=0 and centered isotropic material, their arithmetic is identical
// to the existing IHBE>=11 reference. They do not select a QEPH force law.
using NativeQuadInput = qeph::ReferenceInput;
using NativeQuadReference = qeph::ReferenceData;

struct ResolvedOptions {
  int ihbe=11, irep=0, ismstr=2;
  int nptr=2, npts=2, nptt=1, layers=1;
  int idrill=0, npinch=0, material_law=44, property_type=1;
  int ithick=1, iplas=1;
  double offset_ratio=0;
  double inertia_denominator_override=0;
  double membrane_viscosity=0; // Resolved GROUP_PARAM%VISC_DM, not a force default.
  double numerical_viscosity=0; // Resolved GEO(13) supplied to CNDLENI.
};

struct ReferenceInput {
  NativeQuadInput quadrilateral;
  ResolvedOptions options;
  double initial_a11_pa=0; // Resolved virgin PM(24); material startup owns it.
};

struct StartupCoefficients {
  double characteristic_length_m=0; // CNDLENI ALDT, not a nodal timestep.
  double sound_speed_m_s=0;
  double viscosity_timestep_factor=0;
  double unscaled_element_dt_s=0;
  double nodal_translation_stiffness_n_m=0; // Same contribution at all 4 nodes.
  double nodal_rotation_stiffness_nm=0;
};

// Copyable immutable value; no accepted/trial selector, source owner or clock.
class Reference {
 public:
  TL_QBAT_HD bool prepared() const { return prepared_; }
  TL_QBAT_HD const ReferenceInput& input() const { return input_; }
  TL_QBAT_HD const NativeQuadReference& quadrilateral() const { return quad_; }
  TL_QBAT_HD const StartupCoefficients& coefficients() const { return coefficients_; }
 private:
  ReferenceInput input_{};
  NativeQuadReference quad_{};
  StartupCoefficients coefficients_{};
  bool prepared_=false;
  friend TL_QBAT_HD Status InitializeReference(const ReferenceInput&, Reference&);
};

struct CurrentInput {
  Vec3 position_m[4]{};
  // Only the active current-geometry path is in this value increment.
  // Native OFF=2 (saved small-strain geometry) and removed parents are deferred.
  double native_off=1;
};

struct PointDerivatives {
  double jacobian_m2=0;
  double hx_per_m=0, hy_per_m=0;
  double membrane_b_per_m[8]{}; // CBADEF1 BM(1:8), in native packed order.
};

struct Geometry {
  Matrix3 frame; // Current engine CLSKEW3 axes in world-space columns.
  double area_m2=0, reciprocal_area_per_m2=0;
  double actual_warpage_m=0; // CBACOOR ZL1; NPTT1 still selects flat algorithm.
  Vec3 centered_projected_position_m[4]{};
  double native_vcore[12]{}; // First 4 are 1/m, next 2 unitless, final 6 m.
  PointDerivatives point[4]{}; // IS outer, IR inner: (1,1),(2,1),(1,2),(2,2).
  // CBADEFSH constant shear coefficients acting on corrected V13/V24.
  // Order V13.y,V24.y,V13.x,V24.x; not the point-dependent bilinear shear B.
  double assumed_shear_per_m[4]{};
};
} // namespace tl::fea::qbat
