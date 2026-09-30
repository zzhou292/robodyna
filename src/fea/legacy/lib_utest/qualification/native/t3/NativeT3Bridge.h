#pragma once
#include <cstddef>

namespace tl::qualification::t3::detail {
inline constexpr std::size_t kFrameValues = 13;
inline constexpr std::size_t kMassValues = 26;
extern "C" {
// Column-major X(3,3), then packed row-major frame9, area, x2/x3/y3.
void t3_r1_frame(const double* position, double* output);
// Native local x2/x3/y3; material rho/t; frozen ACOS guard. Status0 alone
// publishes usable scratch. All three cosines are guarded before any ACOS.
void t3_r1_selected_mass(const double* local, const double* area,
                         const double* material, const double* acos_limit,
                         double* output, int* status);
}
}  // namespace tl::qualification::t3::detail
