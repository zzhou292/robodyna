#include "Relations.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
using output::Require;
Relations::Relations(Data& data, const Counts& expected) : data_(data), expected_(expected) {
    data_.relations.reserve(expected.relations);
    data_.slots.reserve(expected.slots);
}
void Relations::Append(Kind kind, Role role, std::uint64_t id, std::uint64_t part,
    std::size_t source_row, const std::size_t* slots, std::size_t count, std::size_t rigid_root) {
    Require(data_.relations.size() < expected_.relations && count &&
        count <= expected_.slots-data_.slots.size() && source_row <= UINT32_MAX && id && slots,
        "Connectivity relation inventory exceeds its exact reservation");
    for (std::size_t slot = 0; slot < count; ++slot)
        Require(slots[slot] < expected_.nodes, "Connectivity relation has an absent domain slot");
    Require(rigid_root == SIZE_MAX || rigid_root < UINT16_MAX, "Connectivity PART root index exceeds scope");
    const auto offset = data_.slots.size();
    for (std::size_t slot = 0; slot < count; ++slot)
        data_.slots.push_back(static_cast<std::uint32_t>(slots[slot]));
    data_.relations.push_back({id,part,static_cast<std::uint32_t>(source_row),
        static_cast<std::uint32_t>(offset),static_cast<std::uint32_t>(count),kind,role,
        static_cast<std::uint16_t>(rigid_root == SIZE_MAX ? UINT16_MAX : rigid_root)});
}
void Relations::Complete() const {
    Require(data_.relations.size() == expected_.relations && data_.slots.size() == expected_.slots,
            "Connectivity omitted a typed relation or support slot");
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
