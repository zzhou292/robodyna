#pragma once
#include "../../native/qeph/QephForceReference.h"
#include "../../native/t3/T3ForceReference.h"
#include <array>
#include <cstddef>

namespace tl::qualification::layered_native {
// Borrowed for one prescribed evaluation; the oracle owns no integration clock.
// Source-selected NIP3/ITHK1/LAW44, isotropic centered shell, VP2 optional.
struct Law44 {
  const double* plastic_strain=nullptr;
  const double* yield_stress=nullptr;
  std::size_t count=0;
  double rate_coefficient_per_s=0, rate_exponent=0, cutoff_hz=0;
};
// Per point: XX,YY,XY,YZ,ZX stress Pa; accumulated PLA; filtered rate 1/s.
using Points=std::array<std::array<double,7>,3>;
static_assert(sizeof(Points)==21*sizeof(double),"Native point packet must be contiguous");
struct SectionDiagnostics {
  double plastic_work_increment_j=0,mean_plastic_strain=0,max_plastic_strain=0;
  double mean_tangent_ratio=0,min_tangent_ratio=0,mean_yield_pa=0,last_yield_pa=0;
  double total_shell_rate_per_s=0;
};
struct QephHistory { qeph::History shell; Points points{}; };
struct T3History { t3::History shell; Points points{}; };
struct QephTrial { qeph::ForceTrial shell; Points points{}; SectionDiagnostics section; };
struct T3Trial { t3::ForceTrial shell; Points points{}; SectionDiagnostics section; };
// The full native kinematics (including frame and local normals), world force/
// couple and complete HOURG/work stay in shell. Caller explicitly accepts a
// returned history; every failure preserves output and borrowed input bytes.
qeph::Status Evaluate(const qeph::Reference&,const QephHistory&,
                      const qeph::PrescribedInterval&,const Law44&,QephTrial&) noexcept;
t3::Status Evaluate(const t3::Reference&,const T3History&,
                    const t3::PrescribedInterval&,const Law44&,T3Trial&) noexcept;
} // namespace tl::qualification::layered_native
