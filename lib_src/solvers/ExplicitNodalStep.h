#pragma once

#include "FENodalState.h"

namespace tl::fea {

enum class NodalStepAdmissionKind {
  Unspecified,
  PrescribedConstantLoads
};

// Explicit coordinator declaration for THIS completed assembly. The owner can
// enforce provenance, the stated step/rotation limit and finite candidate
// state. It cannot infer missing contributors or verify an external nonlinear
// stability argument from a force vector. PrescribedConstantLoads asserts
// state-independent loads and excludes displacement/velocity-dependent shell,
// contact and damping forces. No coupled nonlinear admission exists here yet;
// it must be qualified independently before advancing an elastic coupon.
// Legacy translation rows must be empty for this declared load case and never
// substitute for module candidate checks before Commit.
struct NodalStepAdmission {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  double maximum_dt = 0;
  double maximum_rotation_increment = 0;  // Radians, positive and strictly < pi.
  NodalStepAdmissionKind kind = NodalStepAdmissionKind::Unspecified;
};

// Fixed-step velocity-first x/v and isotropic angular update. WORLD convention:
// omega_new = omega + h * inverse_inertia * couple;
// q_new = QuaternionFromRotationVector(h * omega_new) * q.
// Gyroscopic terms vanish only for the admitted isotropic total inertia.
// All writes are trial data. Fixed component reactions are minus assembled
// force/couple at the accepted base state; no separate physical clock exists.
NodalReport AdvanceNodal(FENodalState&, const NodalTrialToken&, const NodalStepAdmission&);

}  // namespace tl::fea
