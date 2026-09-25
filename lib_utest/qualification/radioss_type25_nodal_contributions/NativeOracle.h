// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Coefficients.h"
#include <array>
namespace type25_contribution_test {
namespace n=tlfea::contact::radioss_type25;
struct NativeSolidObservation {
  std::array<double,8> volume,bulk_volume;
  std::array<double,12> extended_volume,extended_bulk_volume;
};
struct NativeLengthObservation {double value=0;int diagnostics=0;};
// Serial qualification-only original Fortran; no production numerical helper.
NativeSolidObservation OracleSolid(const n::NativeSolidNodalInput&,double seed);
double OracleSpringPrepared(const n::NativeSpringNodalInput&,double prepared_xl,double seed=73.);
NativeLengthObservation OracleLength(n::SpringNodalKind,int length_mode,const std::array<double,6>& endpoints,double noise=0.);
} // namespace type25_contribution_test
