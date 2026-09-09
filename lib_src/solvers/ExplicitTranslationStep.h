#pragma once
#include "FENodalState.h"

namespace tl::fea {
// Non-owning fixed-step operation. Reuses Chrono EulerSemiImplicit's order:
// v_trial = v_accepted + h * M^-1 * F(x_accepted,v_accepted)
// x_trial = x_accepted + h * v_trial.
// Source: chrono/timestepper/ChTimestepperExplicit.cpp, Advance lines 180-208.
// Here all writes remain private trial state until the owner commits once.
// No contact search, material law, shell rotation or physical history is owned.
// Damping is velocity-explicit; the qualified stability contract is fixed h and
// frozen symmetric PSD stiffness/damping. Variable-step/contact-switching safety
// and arbitrary nonlinear tangents require independent qualification.
NodalReport AdvanceTranslations(FENodalState&, const NodalTrialToken&);
}  // namespace tl::fea
