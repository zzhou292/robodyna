#pragma once
#include "Internal.h"
#include "modelio/solid_control/SourceIds.h"
namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
inline std::uint64_t Shift(std::uint64_t value, std::int64_t offset) {
    try { return controls::detail::Shift(value, offset); }
    catch (const controls::detail::Failure& failure) { RejectControl(failure.report); }
}
inline std::int64_t Offset(const modelio::tied_shell::SourceEvidence& value) {
    try { return controls::detail::Offset(value); }
    catch (const controls::detail::Failure& failure) { RejectControl(failure.report); }
}
}
