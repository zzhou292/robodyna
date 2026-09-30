// SPDX-License-Identifier: AGPL-3.0-or-later
// Fixed-size host/device adaptation of OpenRadioss VALPVEC_V (2026 Siemens).
// Full enclosing source and exact extraction are pinned by the qualification.
#pragma once
#include "native_symmetric_eigen3/Roots.h"
#include "native_symmetric_eigen3/Directions.h"
#include "native_symmetric_eigen3/Repeated.h"

namespace tl::math {
// Tensor order XX,YY,ZZ,XY,YZ,ZX; shear entries are true tensor components.
// Unlike SymmetricEigen3, native fallback/order/sign decisions are retained.
// No rescaling or PSD assumption: reject nonfinite native arithmetic and leave
// output unchanged. The selected material caller owns its stretch constraints.
TL_NATIVE_SPECTRUM_HD inline bool NativeSymmetricEigen3(const double (&tensor)[6],
                                             NativeSymmetricSpectrum3& output) noexcept {
  namespace detail = native_symmetric_eigen3;
  for (double value : tensor)
    if (!Finite(value)) return false;
  detail::Work work;
  NativeSymmetricSpectrum3 next;
  if (!detail::Roots(tensor, work, next)) return false;
  switch (detail::Path(work)) {
    case detail::DirectionPath::NearTriple:
      for (unsigned k = 0; k < 3; ++k) next.value[k] = tensor[k];
      detail::CoordinateDirections(next);
      break;
    case detail::DirectionPath::Distinct:
      if (!detail::DistinctDirections(work, next)) return false;
      break;
    case detail::DirectionPath::Repeated:
      if (!detail::RepeatedDirections(work, next)) return false;
      break;
  }
  double* v = next.vectors.v;
  v[2] = v[3]*v[7] - v[6]*v[4];
  v[5] = v[6]*v[1] - v[0]*v[7];
  v[8] = v[0]*v[4] - v[3]*v[1];
  if (work.compression_swap) {
    for (unsigned r = 0; r < 3; ++r) {
      const double saved = v[3*r+2];
      v[3*r+2] = v[3*r];
      v[3*r] = -saved;
    }
  }
  for (double value : next.vectors.v)
    if (!Finite(value)) return false;
  output = next;
  return true;
}
} // namespace tl::math
