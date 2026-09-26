#pragma once
#include "lib_src/collision/RadiossType25Coefficients.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
namespace n = tlfea::contact::radioss_type25;
// Resolved source operands only; this private value seam grants no ownership,
// topology or Ready source authority. Unselected branch payload is not read.
struct MainPacket {
    bool has_solid = false, has_shell = false, copy_partner = false;
    n::NativeInternalSolidMainCoefficientInput solid;
    n::NativeShellMainCoefficientInput shell;
};
struct MainResult { double primary = 0, partner = 0, solid_length = 0; };
n::CoefficientStatus Evaluate(const MainPacket&, MainResult*) noexcept;
}
