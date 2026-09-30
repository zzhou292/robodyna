#pragma once
#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
struct PrimaryValues {
    double primary = 0, partner = 0, characteristic_length = 0;
};
PrimaryValues EvaluateValues(n::NativeShellMainCoefficientInput, n::ShellLayout,
    const n::NativeSolidMainCoefficientInput*);
bool SameValues(const PrimaryValues&, const PrimaryValues&) noexcept;
}
