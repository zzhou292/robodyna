#pragma once
#include <array>
#include <cstddef>

namespace tl::qualification::law44 {
enum class CurveContinuation { StrictDomain, NativeLastSegment };
struct RateInput {
    bool active = false;
    double coefficient_per_s = 0, exponent = 0;
    double total_shell_rate_per_s = 0, filter_coefficient = 1;
    double accepted_filtered_rate_per_s = 0;
};
// Native-only input contract: no production point update or parameter packing.
struct Input {
    double young = 0, poisson = 0, density = 0, transverse_shear_modulus = 0;
    const double* plastic_strain = nullptr;
    const double* yield_stress = nullptr;
    std::size_t point_count = 0;
    std::array<double, 5> accepted_stress{}, strain_increment{};
    double accepted_plastic_strain = 0;
    RateInput rate;
    // Qualification admission only; complete native VINTER is unchanged.
    CurveContinuation continuation = CurveContinuation::StrictDomain;
};
struct Result {
    std::array<double, 5> stress{};
    double plastic_strain = 0, plastic_increment = 0, tangent_ratio = 0;
    double total_thickness_strain = 0, yield_before = 0, sound_speed = 0;
    double equivalent_stress = 0, plastic_work_density = 0;
    double filtered_rate_per_s = 0;
};
// Explicit rate-off or filtered total-rate VP2; isotropic, no failure, no
// nonlocal update. Output changes
// only after finite native results return; no solver owner or clock is created.
bool Evaluate(const Input& input, Result& output);
// Selected HM_READ_MAT/MULAWC expression under the actual native PI constant.
// The caller supplies an already resolved frequency and positive timestep.
double NativeFilterCoefficient(double cutoff_per_s, double dt);
// Selected CZFORC3/C3FORC3 scalar before CMAIN3 changes reported thickness.
double NativeShellRate(const std::array<double, 8>& increment,
                       double accepted_thickness, double dt);
struct SectionRule {
    std::array<double, 3> position{}, membrane_weight{}, moment_weight{};
};
// Actual COQINI/COQINI_WM literal promotion, under the same native compiler
// flags as the point reference. Native COMMON setup is for serial tests only.
SectionRule NativeSectionRule();
}  // namespace tl::qualification::law44
