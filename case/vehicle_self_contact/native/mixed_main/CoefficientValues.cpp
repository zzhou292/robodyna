#include "CoefficientValues.h"
#include "lib_src/math/ScalarBits.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
n::CoefficientStatus Evaluate(const MainPacket& in, MainResult* output) noexcept {
    if (!output || (!in.has_solid && !in.has_shell) || (in.copy_partner && !in.has_shell))
        return n::CoefficientStatus::InvalidInput;
    if (in.has_solid && in.has_shell &&
        !tl::math::SameScalarBits(in.solid.first.scale, in.shell.scale))
        return n::CoefficientStatus::InvalidInput;
    MainResult next;
    if (in.has_solid) {
        n::NativeSolidMainCoefficientResult solid;
        const auto status = in.solid.first.face == n::MainFaceKind::Internal ?
            n::EvaluateNativeInternalSolidMainCoefficient(in.solid, &solid) :
            n::EvaluateNativeReaderSolidMainCoefficient(in.solid.first, &solid);
        if (status != n::CoefficientStatus::Ok) return status;
        next.primary = solid.stiffness; next.solid_length = solid.characteristic_length;
    }
    if (in.has_shell) {
        n::NativeScalarCoefficient shell;
        const auto status = n::EvaluateNativeShellMainCoefficient(in.shell, &shell);
        if (status != n::CoefficientStatus::Ok) return status;
        // Admitted source E,T,scale are positive and the product is representable.
        // Thus this ordinary leaf equals raw STC, including a real shell partner.
        if (!(shell.value > 0.)) return n::CoefficientStatus::UnsupportedProfile;
        next.primary = next.primary > shell.value ? next.primary : shell.value;
        if (in.copy_partner) next.partner = shell.value;
    }
    *output = next;
    return n::CoefficientStatus::Ok;
}
}
