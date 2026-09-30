#pragma once

#include "output/ArtifactIO.h"

namespace crash::benchmarks {

struct CouponProfileConfig {
    unsigned warmup_steps = 10;
    unsigned steps_per_repeat = 100;
    unsigned repetitions = 3;
};

// P0 public-API profile of the existing two-Q4 case. Throws on invalid bounds,
// CUDA/API failure or any unchanged numerical admission rejection. Uses one
// owner and at most 1000 accepted steps; no timing assertion or solver change.
// Spectral diagnostic cadence is explicitly 64 intervals, versus B2's 20.
// Run only under the external workstation guard described in README.md.
void ValidateCouponProfileConfig(const CouponProfileConfig&);
output::Document ProfileElasticCoupon(const CouponProfileConfig&);

}  // namespace crash::benchmarks
