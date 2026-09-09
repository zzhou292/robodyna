#pragma once

#include "FENodalState.h"

namespace tl::fea {

enum class NodalStepAdmissionKind {
  Unspecified,
  PrescribedConstantLoads,
  RestrictedElasticTrajectory,
  RestrictedHistoryTrajectory  // Used only by AdvanceStaggeredHistory.
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
// rows must be empty in every mode: a coupled system must not masquerade as
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

// Declares prescribed constant, state-independent WORLD loads for the new
// staggered operation only. Excludes shell/contact/damping/history forces and
// any donor-startup claim. There is deliberately no elastic/history admission.
struct NodalStaggeredPrescribedAdmission {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  double maximum_dt = 0;
  double maximum_rotation_increment = 0;
};
// Initial physical v0/omega0 are collocated at t0=0. The first successful step
// kicks by h/2, all later steps by h; every drift uses h. Failed trials do not
// consume the initial half kick. Prepared velocity time is base_time+h/2.
// Reuses the same owner, arithmetic, six allocations and reaction convention.
NodalReport AdvanceStaggeredPrescribed(FENodalState&, const NodalTrialToken&,
                                      const NodalStaggeredPrescribedAdmission&);

// Case-qualified fixed-step recurrence with state/history-dependent loads.
// The case must qualify the complete history/velocity recurrence and candidate
// envelope before choosing maximum_dt. No translational PSD row or stateless
// elastic-rate formula establishes that argument. A nonzero qualification ID
// and matching completed validation receipt are mandatory before Commit.
// This operation advances only nodal trial state. The coordinator must stage
// every material participant and publish it infallibly after owner Commit.
struct NodalStaggeredHistoryAdmission {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  double maximum_dt = 0, maximum_rotation_increment = 0;
  std::uint64_t qualification_id = 0;
};
NodalReport AdvanceStaggeredHistory(FENodalState&, const NodalTrialToken&,
                                   const NodalStaggeredHistoryAdmission&);

}  // namespace tl::fea
