// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossingTypes.h"
#include "native/PortableStd.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <boost/version.hpp>
#include <cstring>
#include <limits>

namespace tlfea::contact::represented_interval_crossing {
struct ExactProjectionDomainReport {
  bool supported = false;
  bool eligible = false;
  unsigned sample_depth = 0;
  std::size_t coordinate_bits = 0;
  std::size_t degree_two_bits = 0;
  std::size_t limb_bits = 0;
  std::size_t karatsuba_cutoff = 0;
};

// Private numerical-domain proof, derived only from the actual immutable pair.
// No caller-supplied validity flag or cached foreign-path authority exists.
class ExactProjectionDomain {
 public:
  TL_MATH_HOST_DEVICE static ExactProjectionDomain FromPaths(const RepresentedTrianglePath& first,
                                        const RepresentedTrianglePath& second,
                                        unsigned maximum_cell_depth) noexcept {
    ExactProjectionDomain result;
    auto& report = result.report_;
    report.limb_bits = portable::numeric_limits<boost::multiprecision::limb_type>::digits;
    report.karatsuba_cutoff = boost::multiprecision::backends::karatsuba_cutoff;
    // Re-audit the allocation paths before enabling another dependency version.
    report.supported = BOOST_VERSION == 107400 &&
        (report.limb_bits == 32 || report.limb_bits == 64) &&
        report.karatsuba_cutoff > 1 &&
        report.karatsuba_cutoff - 1 <= SIZE_MAX / report.limb_bits;
    if (!report.supported || maximum_cell_depth > 52) return result;
    report.sample_depth = maximum_cell_depth + 1;
    int minimum = portable::numeric_limits<int>::max();
    int maximum = portable::numeric_limits<int>::min();
    const RepresentedTrianglePath* paths[]{&first, &second};
    for (const auto* path : paths) {
      if (path->motion != RepresentedMotion::LinearNodalV1) return result;
      for (const auto& vertex : path->vertices)
        for (const auto point : vertex.endpoint) {
          const double coordinates[]{point.x, point.y, point.z};
          for (double coordinate : coordinates) {
            std::uint64_t bits = 0; portable::memcpy(&bits, &coordinate, sizeof(bits));
            const auto encoded = static_cast<unsigned>((bits >> 52) & 0x7ffu);
            if (encoded == 0x7ffu) return result;
            // Mirror Exact(double), including zero and unnormalized mantissas.
            const int exponent = encoded ? static_cast<int>(encoded) - 1023 - 52 : -1074;
            if (exponent < minimum) minimum = exponent;
            if (exponent > maximum) maximum = exponent;
          }
        }
    }
    report.coordinate_bits = 53 + static_cast<unsigned>(maximum - minimum) + report.sample_depth;
    report.degree_two_bits = 2 * report.coordinate_bits + 3;
    report.eligible = report.degree_two_bits <=
        (report.karatsuba_cutoff - 1) * report.limb_bits;
    return result;
  }
  TL_MATH_HOST_DEVICE bool eligible() const noexcept { return report_.eligible; }
  TL_MATH_HOST_DEVICE ExactProjectionDomainReport report() const noexcept { return report_; }
 private:
  TL_MATH_HOST_DEVICE ExactProjectionDomain() = default;
  ExactProjectionDomainReport report_;
};
}  // namespace tlfea::contact::represented_interval_crossing
