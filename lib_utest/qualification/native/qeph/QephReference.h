#pragma once

#include "QephReferenceTypes.h"

namespace tl::qualification::qeph {

// Q1 qualification only: one centered isotropic QEPH cell, LAW1 resolved branch
// ISMSTR=-1, ITHK=0, ISROT=0, IREP=0, IVECTOR=0, IRESP=2, IMPL_S=0.
// Convex projected geometry is required; source deactivation and projection
// singularity are failures. QephForceReference.h separately composes the qualified
// Q2 material/history/force operation with this same reference and geometry.
// The operations publish every output together only on success. Input and
// accepted Reference remain unchanged on failure. No allocation or nodal clock.
// Calls to this oracle are internally serialized around its isolated COMMON
// context; this does not make other native qualification libraries thread safe.
Status Initialize(const ReferenceInput& input, Reference& output) noexcept;

// Native temporal contract: endpoint X, midpoint world V/VR, positive finite dt.
// Kinematics receives diagnostics only, never a nodal advance or material state.
Status EvaluatePrescribed(const Reference& reference,
                          const PrescribedInterval& interval,
                          Kinematics& output) noexcept;

}  // namespace tl::qualification::qeph
