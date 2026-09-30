#pragma once
#include "../Values.h"
namespace crash::cases::vehicle_self_contact::native::coated::test {
// Independent small native numerical oracle: max1024nodes/256solids/256shells.
// Serial and non-reentrant: the original IN24 routine uses a private COMMON block.
// Reader consumes declared before-INITIA slots. Classification runs complete
// IN24 with explicit supplied solid order; it does not prove full-case ordering.
std::array<std::uint32_t, 8> NativeReader(ReaderKind,
    const std::array<std::uint32_t, 8>&, const std::vector<Node>&);
std::vector<int> NativeRoles(const Inputs&);
// Original six-word sort only; duplicates remain in this numerical observation.
// Production rejects duplicate keys pending explicit source membership handling.
std::vector<std::uint32_t> NativeOrder(const Inputs&, const std::vector<int>& raw_roles);
}
