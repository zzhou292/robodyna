// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cfenv>
#include <cstdint>
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif

namespace tlfea::contact::self_contact_filters {
// GPU arithmetic is explicitly binary64 RN with gradual underflow. If the host
// public oracle would use another environment, do not silently claim parity.
// An eventual compositor may retain the original CPU path on this status.
inline bool CompatibleHostArithmetic() noexcept {
  if (std::fegetround() != FE_TONEAREST) return false;
#if defined(__x86_64__) || defined(__i386__)
  const unsigned csr = _mm_getcsr();
  std::uint16_t control = 0;
  asm volatile("fnstcw %0" : "=m"(control));
  constexpr unsigned noncanonical = (1u << 15) | (1u << 6) | (3u << 13);
  return (csr & noncanonical) == 0 && (csr & 0x1f80u) == 0x1f80u &&
      (control & 0x0c3fu) == 0x003fu;
#else
  // No guessed FTZ/trap state on another host ABI. This isolated numerical
  // adapter remains unavailable until that platform supplies an audited check.
  return false;
#endif
}
}  // namespace tlfea::contact::self_contact_filters
