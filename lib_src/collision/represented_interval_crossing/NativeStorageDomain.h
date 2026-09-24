// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ExactProjectionDomain.h"

namespace tlfea::contact::represented_interval_crossing {
struct NativeStorageDomainReport {
  ExactProjectionDomainReport projection;
  std::size_t maximum_coefficient_bits = 0;
  std::size_t maximum_product_limbs = 0;
  bool eligible = false;
};
// Full-helper audit: degree <= 4, largest coefficient<384*(2^B)^4;
// RegularCell control<48*(2^B)^2. B <= 125 fits 512 bits including
// general-product limb rounding (4+4 or 6+2 on the audited 64-bit backend).
// See planning/NATIVE_EXACT_STORAGE_DOMAIN_AUDIT_2026-09-24.md.
class NativeStorageDomain {
 public:
  TL_MATH_HOST_DEVICE static NativeStorageDomain FromPaths(const RepresentedTrianglePath& a,
      const RepresentedTrianglePath& b, unsigned maximum_depth) noexcept {
    NativeStorageDomain result;
#if !defined(__GNUC__) && !defined(__clang__)
    // The eligible automatic frame requires the explicitly audited noinline contract.
    return result;
#endif
    result.report_.projection = ExactProjectionDomain::FromPaths(a, b, maximum_depth).report();
    const auto& proof = result.report_.projection;
    if (!proof.supported || !proof.eligible || proof.limb_bits != 64 || proof.karatsuba_cutoff != 40 ||
        !proof.coordinate_bits || proof.coordinate_bits > 125) return result;
    result.report_.maximum_coefficient_bits = 4 * proof.coordinate_bits + 9;
    const auto limbs = [](std::size_t bits) { return (bits + 63) / 64; };
    const auto two_by_two = 2 * limbs(2 * proof.coordinate_bits + 3);
    const auto three_by_one = limbs(3 * proof.coordinate_bits + 5) + limbs(proof.coordinate_bits + 1);
    result.report_.maximum_product_limbs = two_by_two > three_by_one ? two_by_two : three_by_one;
    result.report_.eligible = result.report_.maximum_coefficient_bits <= 512 &&
        result.report_.maximum_product_limbs <= 8;
    return result;
  }
  TL_MATH_HOST_DEVICE bool eligible() const noexcept { return report_.eligible; }
  TL_MATH_HOST_DEVICE NativeStorageDomainReport report() const noexcept { return report_; }
 private:
  TL_MATH_HOST_DEVICE NativeStorageDomain() = default;
  NativeStorageDomainReport report_;
};
}  // namespace tlfea::contact::represented_interval_crossing
