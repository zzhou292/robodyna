#pragma once

#include "FENodalState.h"

namespace tl::fea {

enum class NodalStepAdmissionKind {
  Unspecified,
  PrescribedConstantLoads,
  RestrictedElasticTrajectory
};

// Explicit coordinator declaration for THIS completed assembly. The owner can
// enforce provenance, the stated step/rotation limit and finite candidate
// state. It cannot infer missing contributors or verify an external nonlinear
// stability argument from a force vector. PrescribedConstantLoads asserts
// state-independent loads and excludes displacement/velocity-dependent shell,
// contact and damping forces. RestrictedElasticTrajectory names a separately
// qualified configuration/material/mass envelope and a sampled all-DOF rate
// bound. The coordinator owns its numerical argument and candidate checks;
// this declaration is not a general nonlinear stability theorem. That mode
// requires an explicit validation receipt before Commit. Legacy translation
// rows must be empty in both modes: a coupled system must not masquerade as
// the unrelated frozen translational PSD proof.
struct NodalStepAdmission {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  double maximum_dt = 0;
  double maximum_rotation_increment = 0;  // Radians, positive and strictly < pi.
  NodalStepAdmissionKind kind = NodalStepAdmissionKind::Unspecified;
  std::uint64_t qualification_id = 0;  // Required only for restricted elasticity.
  double stiffness_rate_envelope = 0; // Positive sampled all-DOF norm, s^-2.
};

// Acknowledges completion of the named case's candidate validators. The caller
// checks all tagged element/energy/geometry results before issuing this receipt.
// These values prevent accidental stale/missing validation; like assembly views,
// they cannot authenticate that an external participant actually executed.
struct NodalValidationReceipt {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0, qualification_id = 0;
  bool passed = false;
};

// Fixed-step velocity-first x/v and isotropic angular update. WORLD convention:
// omega_new = omega + h * inverse_inertia * couple;
// q_new = QuaternionFromRotationVector(h * omega_new) * q.
// Gyroscopic terms vanish only for the admitted isotropic total inertia.
// All writes are trial data. Fixed component reactions are minus assembled
// force/couple at the accepted base state; no separate physical clock exists.
NodalReport AdvanceNodal(FENodalState&, const NodalTrialToken&, const NodalStepAdmission&);
NodalReport CompleteNodalValidation(FENodalState&, const NodalTrialToken&, const NodalValidationReceipt&);

}  // namespace tl::fea
