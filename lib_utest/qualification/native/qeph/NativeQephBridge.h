#pragma once

#include <mutex>

// Private binary64, node-major C bindings. No native COMMON/type layout crosses
// this boundary. Public records and validation live in QephReference.cpp.
namespace tl::qualification::qeph::detail {
constexpr int kStartupValues = 34;
constexpr int kKinematicValues = 80;
// Shared by every native reference operation; never a second material context.
std::mutex& NativeContext() noexcept;
extern "C" {
void qeph_q1_startup(const double* x, const double* material, double* output);
void qeph_q1_kinematics(const double* x, const double* v, const double* omega,
                        const double* thickness, const double* dt,
                        double* output, int* planar, int* status);
}
}  // namespace tl::qualification::qeph::detail
