#ifndef ROBODYNA_EXAMPLES_API_SPRING_CHECK_H
#define ROBODYNA_EXAMPLES_API_SPRING_CHECK_H

#include <cmath>
#include <iomanip>
#include <iostream>

namespace robodyna::examples::api {

// Reference for these two small examples only: m=3, k=12, rest length=1,
// initial extension=0.1, zero velocity, damping and gravity.
inline int ReportSpring(const char* model, double time, double position, double velocity) {
    constexpr double final_time = 0.1;
    const double expected = 1 + .1 * std::cos(2 * final_time);
    if (!std::isfinite(time) || !std::isfinite(position) || !std::isfinite(velocity) ||
        std::abs(time - final_time) > 1e-13 || std::abs(position - expected) > 3e-6 || velocity >= 0) {
        std::cerr << "Spring trajectory failed its analytic check\n";
        return 1;
    }
    std::cout << std::setprecision(17)
              << "model=" << model << " backend=cpu time_s=" << time
              << " x_m=" << position << " vx_m_per_s=" << velocity
              << " analytic_x_m=" << expected << '\n';
    return 0;
}

}  // namespace robodyna::examples::api
#endif
