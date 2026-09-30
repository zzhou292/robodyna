#pragma once
#include "Internal.h"
#include "../nodal_seed/MaterialSlots.h"
#include "lib_src/collision/radioss_type25/CoefficientUnits.h"

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
struct MaterialSlots { double bulk = 0, retained_bulk = 0, controlled_bulk = 0; };
inline double RefreshControlledBulk(double bulk, double retained_bulk) {
    if (!n::coefficient_detail::Nonnegative(bulk) || !n::coefficient_detail::Nonnegative(retained_bulk))
        Reject(Status::NonfiniteResult, "Invalid native material slots for the controlled-bulk refresh");
    // GNU native MAX keeps the later equal operand, including signed zero.
    // Keep this local; the frozen legacy coefficient Max has another tie rule.
    const double maximum = bulk > retained_bulk ? bulk : retained_bulk;
    const double result = 2. * maximum;
    if (!n::coefficient_detail::Finite(result))
        Reject(Status::NonfiniteResult, "Nonfinite native controlled-bulk refresh");
    return result;
}

// Actual phase: HM_READ_MAT's PM100 snapshot followed by UPDMAT. This does
// not alter a constitutive material or reuse physical M/J or structural STI.
inline MaterialSlots Slots(const modelio::solid_source::Part& part, n::UnitScale units) {
    const auto factors = seed::detail::Factors(units);
    double bulk_pa = 0, retained_pa = 0;
    using Law = modelio::solid_source::MaterialLaw;
    switch (part.material_law) {
    case Law::Law36:
        bulk_pa = retained_pa = part.law36.bulk_pa;
        break;
    case Law::Law42:
        bulk_pa = seed::detail::Law42ContactPm32Pa(part.law42);
        retained_pa = part.law42.bulk_pa;
        break;
    case Law::Law44:
        bulk_pa = retained_pa = part.law44.bulk_pa;
        break;
    case Law::Law90:
        bulk_pa = part.law90.updated().bulk_pa;
        retained_pa = part.law90.reader().contact_bulk_pa;
        break;
    default:
        Reject(Status::UnsupportedSource, "No qualified controlled material-slot association");
    }
    MaterialSlots result;
    result.bulk = bulk_pa / factors.pressure;
    result.retained_bulk = retained_pa / factors.pressure;
    // Original UPDMAT refresh, after the material-specific updater. In
    // particular, LAW90's PM100 remains its earlier reader contact bulk.
    result.controlled_bulk = RefreshControlledBulk(result.bulk, result.retained_bulk);
    if (!n::coefficient_detail::Nonnegative(result.bulk) ||
        !n::coefficient_detail::Nonnegative(result.retained_bulk) ||
        !n::coefficient_detail::Nonnegative(result.controlled_bulk))
        Reject(Status::NonfiniteResult, "Native controlled material slots are invalid or nonfinite");
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
