#pragma once
#include "../TiedShellPacking.h"

namespace crash::modelio::tied_shell::packing_detail {
std::size_t Preflight(const source::CanonicalData&, const Data&, PackingLimits);
void CheckPolicy(const Data&);
PackingData Build(const source::CanonicalData&, const Data&, PackingLimits);
} // namespace crash::modelio::tied_shell::packing_detail
