#pragma once
#include <array>
#include <cstddef>

namespace tl::qualification::law44 {
// Native-only input contract: no production point update or parameter packing.
struct Input {
    double young = 0, poisson = 0, density = 0, transverse_shear_modulus = 0;
    const double* plastic_strain = nullptr;
    const double* yield_stress = nullptr;
    std::size_t point_count = 0;
    std::array<double, 5> accepted_stress{}, strain_increment{};
    double accepted_plastic_strain = 0;
};
struct Result {
    std::array<double, 5> stress{};
    double plastic_strain = 0, plastic_increment = 0, tangent_ratio = 0;
    double total_thickness_strain = 0, yield_before = 0, sound_speed = 0;
    double equivalent_stress = 0, plastic_work_density = 0;
};
// Explicit rate-off, isotropic, no failure, no nonlocal update. Output changes
// only after finite native results return; no solver owner or clock is created.
bool Evaluate(const Input& input, Result& output);
struct SectionRule {
    std::array<double, 3> position{}, membrane_weight{}, moment_weight{};
};
// Actual COQINI/COQINI_WM literal promotion, under the same native compiler
// flags as the point reference. Native COMMON setup is for serial tests only.
SectionRule NativeSectionRule();
}  // namespace tl::qualification::law44
