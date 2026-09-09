#pragma once

#include "QephHistory.h"
#include "QephReference.h"

namespace tl::qualification::qeph {

struct ForceDiagnostics {
  double effective_thickness=0;
  double native_sound_speed=0;
  double membrane_viscosity=0;     // Resolved DM=.015, native defaults experiment.
  double stabilization_viscosity=0;// Resolved DN=.015, native defaults experiment.
  // Native CNDT3 before FACN scatter. Diagnostics, not a stiffness certificate.
  double translational_stiffness=0; // N/m.
  double rotational_stiffness=0;    // N*m.
  double unscaled_element_dt=0;    // DTEL, s, explicit DTFAC1(3)=1.
  std::array<double,2> internal_work_increment{};
  double hourglass_viscous_work_increment=0;
};

struct ForceTrial {
  History proposed_history;
  Kinematics kinematics; // Same pre-CNDT3 geometry/rates as the Q1 operation.
  std::array<Vec3,4> internal_force{};  // Native positive internal force, N.
  std::array<Vec3,4> internal_couple{}; // Native positive world couple, N*m.
  ForceDiagnostics diagnostics;
};

// One centered isotropic LAW1 cell, native CVIS=1, DM=DN=.015. No runtime
// damping switch or production integration. CUPDTN3 subtracts these internal
// forces/couples from nodal RHS. The full native history is proposed together.
// Require an exactly matching reference, interval.base_time==base.stamp.time,
// next sample_index, and a finite representably advancing endpoint time.
// Every failure preserves all base and output bytes. The caller accepts history
// explicitly by value; this library does not own or advance a dynamics clock.
Status EvaluateForce(const Reference&,const History& base,
                     const PrescribedInterval&,ForceTrial&) noexcept;

}  // namespace tl::qualification::qeph
