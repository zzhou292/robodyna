#pragma once
#include "Internal.h"

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
// Contact input slot of the already-admitted V5 alpha2/no-Prony LAW42 source.
// HM_READ_MAT42 PARMAT1=GS survives the GammaInf1 updater as PM32. The
// separate physical bulk field is PM100. This accessor is not a material
// initializer or a claim that an arbitrary Parameters value has source authority.
inline double Law42ContactPm32Pa(const tl::material::law42::Parameters& material) {
    const double result = 2. * material.mu_pa;
    Require(n::coefficient_detail::Positive(material.mu_pa) &&
        n::coefficient_detail::Finite(result), "Invalid selected LAW42 contact PM32");
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
