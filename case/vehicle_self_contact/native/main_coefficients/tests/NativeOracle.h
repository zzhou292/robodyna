#pragma once
#include "../Internal.h"
#include <array>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
struct NativeCandidate {
    n::ShellLayout layout = n::ShellLayout::Quad4;
    std::array<unsigned, 4> nodes{}; // Zero-based qualification-node IDs.
    double thickness = 0, young = 0;
};
// Supplied family storage order is explicit. This serial-only original-source
// oracle observes both NEL/NELTG and reports physical input ordinal, not a native
// vehicle ordinal or a production source authority.
struct NativeResult { std::size_t q4 = SIZE_MAX, t3 = SIZE_MAX, selected = SIZE_MAX; };
NativeResult NativeSupport(const std::vector<NativeCandidate>& storage, const std::array<unsigned, 4>& main);
std::vector<unsigned> NativeOrder(const std::vector<std::array<std::uint32_t, 8>>& keys);
}
