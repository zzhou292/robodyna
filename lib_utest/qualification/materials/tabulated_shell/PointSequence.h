#pragma once
#include "lib_src/materials/TabulatedShellPlasticity.h"

#if defined(__CUDACC__)
#define TL_POINT_TEST_HD __host__ __device__
#else
#define TL_POINT_TEST_HD
#endif
namespace tl::material::point_test {
struct SequenceResult {
  TabulatedShellPlasticityResult endpoint{};
  TabulatedShellPlasticityStatus status = TabulatedShellPlasticityStatus::Ok;
  unsigned accepted_intervals = 0;
};
TL_POINT_TEST_HD inline SequenceResult RunSequence(unsigned case_id) {
  const double curve_x[]{0, .1, .3}, curve_y[]{270e6, 340e6, 362e6};
  TabulatedShellPlasticityParameters p;
  SequenceResult result;
  result.status = PrepareTabulatedShellPlasticity(200e9, .3, 7890, {curve_x, curve_y, 3}, p);
  if (result.status != TabulatedShellPlasticityStatus::Ok) return result;
  TabulatedShellPlasticityHistory base;
  for (unsigned step = 0; step < 64; ++step) {
    TabulatedShellPlasticityInput input;
    const double sign = (step/16)%2 == 0 ? 1. : -1.;
    input.transverse_shear_modulus = p.shear_modulus*5./6.;
    input.strain_increment[0] = sign*(case_id + 1)*.0001;
    input.strain_increment[1] = -.3*input.strain_increment[0];
    input.strain_increment[2] = sign*case_id*.00003;
    input.strain_increment[3] = sign*.00002;
    input.strain_increment[4] = -sign*.00001;
    if (case_id == 7) input.transverse_shear_modulus = -1.;
    result.status = UpdateTabulatedShellPlasticity(p, base, input, result.endpoint);
    if (result.status != TabulatedShellPlasticityStatus::Ok) return result;
    base = result.endpoint.history;
    ++result.accepted_intervals;
  }
  return result;
}
} // namespace tl::material::point_test
#undef TL_POINT_TEST_HD
