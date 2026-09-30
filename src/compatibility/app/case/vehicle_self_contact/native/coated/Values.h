#pragma once
#include "Types.h"
namespace crash::cases::vehicle_self_contact::native::coated {
// Numerical/source value seams for small independent native coupons. They do
// not authenticate caller-made arrays or add a physical source owner.
std::array<std::uint32_t, 8> ReaderSlots(ReaderKind,
    const std::array<std::uint32_t, 8>& declared_nodes, const std::vector<Node>&);
Classification Classify(const Inputs&);
Order SurfaceOrder(const Inputs&, const Classification&);
s::ShellSideRole SideRole(RoleState);
} // namespace crash::cases::vehicle_self_contact::native::coated
