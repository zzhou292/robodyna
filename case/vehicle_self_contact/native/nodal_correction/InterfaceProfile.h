#pragma once
#include "Internal.h"
#include "modelio/solid_control/InterfaceProfile.h"
namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
inline void CheckInterfaceKeyword(const std::string& keyword, InterfaceCensus& census,
                                 const std::string& file, std::size_t line) {
    try { controls::detail::CheckInterfaceKeyword(keyword, census, file, line); }
    catch (const controls::detail::Failure& failure) { RejectControl(failure.report); }
}
}
