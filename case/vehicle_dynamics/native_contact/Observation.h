#pragma once
#include "lib_src/collision/RadiossType25Transaction.h"

namespace crash::cases::vehicle_dynamics::native_contact {
namespace native = tlfea::contact::radioss_type25;
// Value diagnostics for one completed accepted-force/candidate attempt. Contains
// no publication authority, M2 receipt or independent physical clock.
struct Observation {
    bool enabled = false;
    tl::fea::NodalStamp force_base;
    std::uint64_t attempt = 0;
    native::TransactionSourceInfo source;
    native::TransactionDiagnostics diagnostics;
};
} // namespace crash::cases::vehicle_dynamics::native_contact
