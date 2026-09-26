// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
namespace type25_seed_test {
struct NativeObservation {
  std::vector<n::NativeNodalSeed> seeds;
  std::vector<double> young_thickness;
  std::vector<int> shell_incidence;
  std::vector<n::NativeNodalCoefficientResult> finalized;
};
// Complete independent native ordered contact-channel gathering. Inputs are
// unsorted parent packets; native MY_ORDERS and original SPMD_MSIN blocks own
// reference sorting/reduction. ASSTIFI is the existing compiled native oracle.
NativeObservation Oracle(const Case&);
} // namespace type25_seed_test
