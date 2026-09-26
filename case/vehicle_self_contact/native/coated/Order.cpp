#include "Values.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <climits>
#include <numeric>
namespace crash::cases::vehicle_self_contact::native::coated {
s::ShellSideRole SideRole(RoleState state) {
    if (state == RoleState::Ordinary) return s::ShellSideRole::Ordinary;
    if (state == RoleState::CoatingForward) return s::ShellSideRole::CoatingForward;
    output::Require(state == RoleState::CoatingReversed, "Unresolved coating role cannot become a primary");
    return s::ShellSideRole::CoatingReversed;
}
Order SurfaceOrder(const Inputs& input, const Classification& classification) {
    output::Require(classification.contact_complete && classification.roles.size() == input.shells.size(),
                    "Surface ordering requires every selected contact role to resolve");
    output::Require(classification.physical.shells == input.shells.size(), "Coating physical role count differs");
    const auto selected_count = std::count_if(input.shells.begin(), input.shells.end(),
        [](const auto& shell) { return shell.contact_selected; });
    output::Require(std::size_t(selected_count) == classification.contact.shells, "Coating selected role count differs");
    std::uint64_t previous = 0;
    for (const auto& node : input.nodes) {
        output::Require(node.source_id > previous && node.source_id <= INT_MAX,
                        "Case-declared ITAB requires positive representable strictly increasing source NIDs");
        previous = node.source_id;
    }
    output::Require(input.nodes.size() <= std::size_t(INT_MAX) && input.shells.size() <= UINT32_MAX,
                    "Case-declared native node or shell ordinal is not representable");
    struct Key { std::array<std::uint32_t, 6> words{}; std::uint32_t physical = 0; };
    std::vector<Key> keys;
    keys.reserve(classification.contact.shells);
    output::Require(keys.capacity() <= 2*classification.contact.shells, "Surface key allocation exceeds reserved capacity");
    for (std::size_t i = 0; i < input.shells.size(); ++i) {
        const auto& shell = input.shells[i];
        if (!shell.contact_selected) continue;
        Key key;
        key.physical = std::uint32_t(i);
        for (unsigned slot = 0; slot < 4; ++slot) {
            output::Require(shell.primary.nodes[slot] < input.nodes.size(), "Surface key node is outside ITAB");
            key.words[slot] = shell.primary.nodes[slot] + 1;
        }
        const int ordinary = shell.primary.layout == n::ShellLayout::Triangle3 ? 7 : 3;
        const auto state = classification.roles[i].state;
        const auto role = SideRole(state);
        const std::int32_t raw = role == s::ShellSideRole::Ordinary ? ordinary :
            role == s::ShellSideRole::CoatingForward ? ordinary + 1 : -(ordinary + 1);
        key.words[4] = std::uint32_t(raw); // MY_ORDERS treats every INTEGER word unsigned.
        key.words[5] = 0; // Explicit single-surface declaration, IMBIN=0; no mask merge.
        keys.push_back(key);
    }
    // The final ordinal tie implements MY_ORDERS' stable source insertion order
    // without stable_sort's unaccounted temporary allocation.
    std::sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) {
        return a.words < b.words || (a.words == b.words && a.physical < b.physical);
    });
    for (std::size_t i = 1; i < keys.size(); ++i)
        output::Require(!std::equal(keys[i-1].words.begin(), keys[i-1].words.begin()+5, keys[i].words.begin()),
                        "Duplicate native surface key requires explicit source membership resolution");
    Order result;
    result.primary_to_physical.reserve(keys.size());
    result.physical_to_primary.assign(input.shells.size(), UINT32_MAX);
    output::Require(result.primary_to_physical.capacity() <= 2*keys.size() &&
        result.physical_to_primary.capacity() <= 2*input.shells.size(), "Surface order allocation exceeds reserved capacity");
    for (const auto& key : keys) {
        result.physical_to_primary[key.physical] = std::uint32_t(result.primary_to_physical.size());
        result.primary_to_physical.push_back(key.physical);
    }
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
