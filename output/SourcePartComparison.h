#pragma once
#include "ArtifactIO.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace crash::output::source_comparison {
// Shared arithmetic for the original 117-node part's frozen refinement gates.
// Call after the experiment's accepted reader has validated finite fields and
// unit quaternions. Energy, contact, identity and promotion rules remain with
// each experiment; this helper has no solver or CUDA dependency.
inline const Value& Field(const Value& document, const char* name) {
    Require(document.IsObject() && document.HasMember(name), "Missing comparison field");
    return document[name];
}

inline double VectorDifference(const Value& a, const Value& b, const char* field, double scale) {
    const auto& x = Field(a, field);
    const auto& y = Field(b, field);
    Require(x.IsArray() && y.IsArray() && x.Size() == 351 && y.Size() == 351,
            "Wrong source vector size");
    double result = 0;
    for (unsigned n = 0; n < 117; ++n) {
        double square = 0;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const auto j = 3*n + axis;
            Require(x[j].IsNumber() && y[j].IsNumber(), "Invalid vector component");
            const auto difference = x[j].GetDouble() - y[j].GetDouble();
            Require(std::isfinite(difference), "Nonfinite vector difference");
            square += difference*difference;
        }
        result = std::max(result, std::sqrt(square)/scale);
    }
    return result;
}

inline double RotationDifference(const Value& a, const Value& b) {
    const auto& x = Field(a, "orientation_wxyz");
    const auto& y = Field(b, "orientation_wxyz");
    Require(x.IsArray() && y.IsArray() && x.Size() == 468 && y.Size() == 468,
            "Wrong source rotation size");
    double result = 0;
    for (unsigned n = 0; n < 117; ++n) {
        long double p[4]{}, q[4]{};
        for (unsigned j = 0; j < 4; ++j) {
            p[j] = x[4*n+j].GetDouble();
            q[j] = y[4*n+j].GetDouble();
        }
        const long double w = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
        const long double rx = p[0]*q[1] - p[1]*q[0] - p[2]*q[3] + p[3]*q[2];
        const long double ry = p[0]*q[2] + p[1]*q[3] - p[2]*q[0] - p[3]*q[1];
        const long double rz = p[0]*q[3] - p[1]*q[2] + p[2]*q[1] - p[3]*q[0];
        const double angle = static_cast<double>(
            2*std::atan2(std::sqrt(rx*rx + ry*ry + rz*rz), std::abs(w)));
        Require(std::isfinite(angle), "Nonfinite rotation difference");
        result = std::max(result, angle/.01);
    }
    return result;
}

// Position / 1 mm, relative orientation / .01 rad, endpoint velocity / 1 m/s,
// and endpoint angular velocity / 100 rad/s. Raw midpoint arrays are untouched.
inline std::array<double, 4> Kinematics(const Value& a, const Value& b) {
    const auto& time_a = Field(a, "accepted_time_s");
    const auto& time_b = Field(b, "accepted_time_s");
    Require(time_a.IsNumber() && time_b.IsNumber() &&
            std::isfinite(time_a.GetDouble()) && std::isfinite(time_b.GetDouble()) &&
            Bits(time_a.GetDouble()) == Bits(time_b.GetDouble()), "Different physical sample times");
    return {VectorDifference(a, b, "position_xyz_m", .001), RotationDifference(a, b),
            VectorDifference(a, b, "synchronized_velocity_xyz_m_per_s", 1),
            VectorDifference(a, b, "synchronized_omega_world_xyz_rad_per_s", 100)};
}
} // namespace crash::output::source_comparison
