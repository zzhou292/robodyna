#pragma once
#include "NativePhysicalThickness.h"

namespace tl::qualification::law44 {
struct AnalyticInput {
  Input point; // Curve fields must be empty; positive-C/P filtered VP2 only.
  double initial_yield=0,tangent_modulus=0;
};
struct AnalyticResult : PhysicalThicknessResult {
  double native_plastic_hardening=0;
};
// MFUNC=NVARTMP=0. Independent native coefficient preparation; caller keeps
// its own accepted history. All output bytes remain unchanged on rejection.
bool EvaluateAnalytic(const AnalyticInput&,double layer_thickness,double running_thickness,AnalyticResult&);
} // namespace tl::qualification::law44
