#pragma once

#include "lib_src/elements/ShellBatchPublication.h"

namespace crash::cases::vehicle_dynamics::detail {

inline tl::fea::ShellPhysicalScratchReceiptRoster ComposeScratchReceipts(
    const tl::fea::ShellPhysicalScratchReceiptRoster& wall,
    const tl::fea::ShellPhysicalScratchReceiptRoster& self_contact) noexcept {
    return {wall.mapped_wall, self_contact.self_contact};
}

}  // namespace crash::cases::vehicle_dynamics::detail
