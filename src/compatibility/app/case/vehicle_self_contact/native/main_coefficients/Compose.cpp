#include "Compose.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace {
void Check(n::CoefficientStatus status) {
    if (status == n::CoefficientStatus::Ok) return;
    Reject(status == n::CoefficientStatus::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidInput,
        "Native main coefficient rejected resolved source operands");
}
}
PrimaryValues EvaluateValues(n::NativeShellMainCoefficientInput shell, n::ShellLayout layout,
    const n::NativeSolidMainCoefficientInput* solid) {
    shell.layout = layout;
    if (solid) {
        shell.face = n::MainFaceKind::Coating;
        n::NativeCoatedMainCoefficientResult value;
        Check(n::EvaluateNativeCoatedMainCoefficient({shell, *solid}, &value));
        return {value.primary_stiffness, value.partner_stiffness, value.solid_characteristic_length};
    }
    shell.face = n::MainFaceKind::OrdinaryExterior;
    n::NativeScalarCoefficient value;
    Check(n::EvaluateNativeShellMainCoefficient(shell, &value));
    // The admitted E and T are strictly positive, so the ordinary initial
    // MAX(+0, STC) is the same STC copied by I25GAPM to its encoded partner.
    return {value.value, value.value, 0.};
}
bool SameValues(const PrimaryValues& a, const PrimaryValues& b) noexcept {
    return tl::math::SameScalarBits(a.primary, b.primary) &&
        tl::math::SameScalarBits(a.partner, b.partner) &&
        tl::math::SameScalarBits(a.characteristic_length, b.characteristic_length);
}
}
