#pragma once
#include "../CoatedSource.h"
#include "Values.h"
#include "output/BoundedArrayIO.h"
namespace crash::cases::vehicle_self_contact::native::coated::detail {
const source::CanonicalData& CheckSource(const PhysicalModel&, const selection::OriginalSelection&, Config, Limits);
Inputs PrepareInputs(const PhysicalModel&, const selection::OriginalSelection&, const std::string&, Config, Limits);
Digest InputDigest(const Inputs&, const Classification&, const Order*, Config, const std::string& source_binding, std::size_t);
Digest OutputDigest(const s::Snapshot&, const std::string&, std::size_t);
Location ShellLocation(const Inputs&, std::size_t physical);
} // namespace crash::cases::vehicle_self_contact::native::coated::detail
