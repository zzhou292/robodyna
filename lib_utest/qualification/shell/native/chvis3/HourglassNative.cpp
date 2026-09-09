#include "HourglassNative.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>

extern "C" int crash_chvis3_native_impl(int, int, const double*, const double*,
                                       const double*, const double*, const double*,
                                       double*, double*, double*, double*);

namespace {
std::mutex native_context_mutex;
bool Finite(const double* values, std::size_t size) {
  return std::all_of(values, values + size, [](double value) { return std::isfinite(value); });
}
}

extern "C" int crash_chvis3_native(int n, int ismstr, const double* controls,
                                  const double* fields, const double* local_v,
                                  const double* local_w, const double* hour_in,
                                  double* hour_out, double* restoring_force,
                                  double* restoring_couple, double* work_increment) {
  if (n < 1 || n > 16 || (ismstr != 1 && ismstr != 2) || !controls || !fields ||
      !local_v || !local_w || !hour_in || !hour_out || !restoring_force ||
      !restoring_couple || !work_increment) return 1;
  if (!Finite(controls, 3) || !Finite(fields, n * CHVIS_FIELD_COUNT) ||
      !Finite(local_v, n * 12) || !Finite(local_w, n * 12) || !Finite(hour_in, n * 5)) return 1;
  for (int c = 0; c < 3; ++c) if (controls[c] < 0) return 1;
  for (int e = 0; e < n; ++e) {
    auto field = [&](int component) { return fields[component * n + e]; };
    if (!(field(CHVIS_AREA) > 0 && field(CHVIS_THICKNESS) > 0 && field(CHVIS_YOUNG) > 0 &&
          field(CHVIS_RHO) > 0 && field(CHVIS_SSP) > 0 && field(CHVIS_DT) > 0 &&
          field(CHVIS_NU) > -1 && field(CHVIS_NU) < .5 && field(CHVIS_SHF) >= 0)) return 1;
    for (int c = CHVIS_H1; c <= CHVIS_SRH3; ++c) if (field(c) < 0) return 1;
    const double squared = field(CHVIS_PX1)*field(CHVIS_PX1) + field(CHVIS_PX2)*field(CHVIS_PX2)
                         + field(CHVIS_PY1)*field(CHVIS_PY1) + field(CHVIS_PY2)*field(CHVIS_PY2);
    if (!(squared > 0) || !std::isfinite(squared)) return 1;
  }
  std::array<double, 16 * 5> hour{};
  std::array<double, 16 * 12> forces{}, couples{};
  std::array<double, 16> work{};
  int status;
  {
    std::lock_guard<std::mutex> lock(native_context_mutex);
    status = crash_chvis3_native_impl(n, ismstr, controls, fields, local_v, local_w,
                                     hour_in, hour.data(), forces.data(), couples.data(), work.data());
  }
  if (status != 0 || !Finite(hour.data(), n*5) || !Finite(forces.data(), n*12) ||
      !Finite(couples.data(), n*12) || !Finite(work.data(), n)) return 2;
  std::copy_n(hour.data(), n*5, hour_out);
  std::copy_n(forces.data(), n*12, restoring_force);
  std::copy_n(couples.data(), n*12, restoring_couple);
  std::copy_n(work.data(), n, work_increment);
  return 0;
}
