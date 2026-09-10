#pragma once
#include "lib_src/collision/Q4ContactBounds.h"
#include <cstddef>

namespace crash::cases::wall_penalty {
namespace contact=tlfea::contact;
// Startup value reduction only: physical uniform +X translation and zero spin.
// Callers explicitly choose and name the physical mass metric and its ordering.
// In particular, native member masses and aggregate group masses are different
// inputs; this utility never interprets a constrained inverse or live velocity.
struct UniformTranslationKinetic {
    contact::Q4IntegralInterval speed_squared,energy;
    double nominal_speed_squared=0,nominal_energy=0;
    std::size_t mass_count=0;
};
bool BeginUniformTranslation(double speed,UniformTranslationKinetic*) noexcept;
bool AddTranslationMass(double mass,UniformTranslationKinetic*) noexcept;
bool FinishUniformTranslation(const UniformTranslationKinetic&,contact::Q4CertifiedIntegral*) noexcept;
} // namespace crash::cases::wall_penalty
