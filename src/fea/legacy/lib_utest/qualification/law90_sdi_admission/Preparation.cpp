// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Observation.h"
#include "../law90_preparation/native/NativeOracle.h"
#include <cmath>
extern "C" void law90_sdi_units(const double*, const double[3], double*);

namespace law90_sdi {
void Prepare(Observation& out) {
  double values[13]{}; int flags[3]{};
  const char* names[13] = {"MAT_RHO", "Refer_Rho", "MAT_E0", "MAT_NU",
      "LSD_MAT83_ED", "LSD_MAT83_TC", "Hys", "MAT_SHAPE", "MAT_ALPHA",
      "FscaleL", "FscaleL", "EpsilondotL", "Fcut"};
  for (unsigned i = 0; i < 13; ++i) {
    const auto& f = Get(out.fields, names[i]);
    const double raw = i == 10 ? 1.0 : f.value;
    law90_sdi_units(&raw, f.dimension.data(), &values[i]);
  }
  flags[0] = static_cast<int>(Get(out.fields, "Ismooth").value);
  flags[1] = static_cast<int>(Get(out.fields, "MAT_TFLAG").value);
  flags[2] = static_cast<int>(Get(out.fields, "LSD_MAT83_FAIL").value);
  // Native reader uses raw function ordinates with its dimensioned YFAC. Do
  // not convert knots first and change subtraction/multiplication ordering.
  double x[28], y[28]; const int n = 28;
  for (int i = 0; i < n; ++i) { x[i] = out.curve[2*i]; y[i] = out.curve[2*i+1]; }
  law90_native_prepare(values, flags, x, y, &n, out.prepared.data());
  for (double v : out.prepared) Require(std::isfinite(v), "nonfinite native preparation");
}
} // namespace law90_sdi
