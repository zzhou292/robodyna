#pragma once
#include "Fixture.h"
#include "lib_src/constraints/tied_shell/TiedPatchCoefficients.h"

namespace tied_patch_test {
inline tie::CoefficientInput Coefficients(unsigned mode) {
  tie::CoefficientInput input{{.03, 1.8e-6, 1.7e6, 135.}, {2e-6, 3e-6, 5e-6, 5e-6}};
  if (mode < 4) input.initial_master_inertia[mode] = 0;
  if (mode == 5) input.secondary.inertia = 0;
  if (mode == 6) input.secondary.mass = 0;
  if (mode == 7) input.secondary = {};
  return input;
}
inline std::array<double,23> CoefficientValues(const tie::CoefficientTransfer& input,
                                              bool repeated_node) {
  std::array<double,23> values{};
  auto put = [&](unsigned at, const tie::NodalCoefficients& c) {
    values[at] += c.mass;
    values[at+1] += c.inertia;
    values[at+2] += c.translational_stiffness;
    values[at+3] += c.rotational_stiffness;
  };
  for (unsigned i = 0; i < 4; ++i) put(repeated_node && i == 3 ? 8 : 4*i, input.master[i]);
  put(16, input.dependent);
  values[20] = input.source_mass;
  values[21] = input.source_inertia;
  values[22] = input.numerical_mass_delta;
  return values;
}
} // namespace tied_patch_test
