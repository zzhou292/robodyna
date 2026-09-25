// SPDX-License-Identifier: AGPL-3.0-or-later
// I25DST3_3 general geometry/normal interpolation, source a62b27e6.
#pragma once
#include "Prepare.h"
namespace tlfea::contact::radioss_type25::geometry_detail {
TL_MATH_HOST_DEVICE inline Vector InterpolateNormal(Vector normal, Vector plane, double s) {
  const double projection = v::Dot(plane, normal), squared = v::Dot(normal, normal);
  if (projection < 0 || 2. * projection * projection < squared) {
    const double correction = ::sqrt(Max(0., squared - projection * projection)) - projection;
    normal = v::Add(normal, v::Scale(plane, correction));
  }
  normal = Normalize(normal, native_constant::em30);
  normal = v::Add(v::Scale(normal, 1. - s), v::Scale(plane, s));
  return Normalize(normal, native_constant::em30);
}
TL_MATH_HOST_DEVICE inline void General(const NativeGeometryInput& in, Work& w,
                                       NativeRawGeometryResult& out) {
  if (w.bb <= 0) {
    w.closest = Closest(in, w); w.closest_defined = true;
    const auto difference = v::Subtract(in.secondary, w.closest);
    const double distance = ::sqrt(v::Dot(difference, difference));
    out.normal = distance > em03 ? v::Scale(difference, 1. / distance) : w.plane;
    out.geometric_penetration = Max(0., w.gap - distance);
    out.distance = distance;
    return;
  }
  out.normal = w.plane;
  if (!w.triangle) {
    if (w.la < epseg && (in.neighbors[w.sector] != 0 || !w.shell_contact)) {
      out.normal = InterpolateNormal(w.normal[w.sector], w.plane, w.la / epseg);
      w.closest = Closest(in, w); w.closest_defined = true;
    }
  } else {
    const bool a = w.la < epseg && (in.neighbors[0] != 0 || !w.shell_contact);
    const bool b = in.lb < epseg && (in.neighbors[1] != 0 || !w.shell_contact);
    const bool c = in.lc < epseg && (in.neighbors[3] != 0 || !w.shell_contact);
    if (a || b || c) {
      double ax, bx, cx, s;
      if (a) { const double sum = in.lb + in.lc;
        ax = 0; bx = in.lb / sum; cx = in.lc / sum; s = w.la / epseg;
      } else if (b) { const double sum = w.la + in.lc;
        ax = w.la / sum; bx = 0; cx = in.lc / sum; s = in.lb / epseg;
      } else { const double sum = w.la + in.lb;
        ax = w.la / sum; bx = in.lb / sum; cx = 0; s = in.lc / epseg;
      }
      const auto normal = v::Add(v::Add(v::Scale(w.normal[w.a], bx + cx - ax),
          v::Scale(w.normal[w.b], ax + cx - bx)), v::Scale(w.normal[4], ax + bx - cx));
      out.normal = InterpolateNormal(normal, w.plane, s);
      w.closest = Closest(in, w); w.closest_defined = true;
    }
  }
  out.geometric_penetration = Max(0., w.gap + w.bb);
  out.distance = w.bb;
}
} // namespace tlfea::contact::radioss_type25::geometry_detail
