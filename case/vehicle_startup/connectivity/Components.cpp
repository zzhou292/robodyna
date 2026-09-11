#include "Components.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::connectivity {
const char* Name(Kind kind) noexcept {
    constexpr const char* names[]{"qeph","t3","qbat","solid18","solid24","solid6z",
        "type13","type25","part_root","plain_group","cin","point_mass"};
    const auto index = static_cast<std::size_t>(kind);
    return index < KindCount ? names[index] : "invalid";
}
namespace detail {
using output::Require;
Components::Components(tl::util::ConstView<tl::fea::NodalDomainNode> nodes) : nodes_(nodes) {
    Require(nodes.size() && nodes.size() <= UINT32_MAX, "Connectivity node extent is invalid");
    parent_.resize(nodes.size());
    for (std::size_t node = 0; node < nodes.size(); ++node) {
        Require(nodes[node].source_id != 0, "Connectivity needs original nonzero NIDs");
        parent_[node] = static_cast<std::uint32_t>(node);
    }
}
std::uint32_t Components::Root(std::uint32_t node) {
    auto root = node;
    while (parent_[root] != root) root = parent_[root];
    while (parent_[node] != node) {
        const auto next = parent_[node];
        parent_[node] = root;
        node = next;
    }
    return root;
}
void Components::Join(const std::uint32_t* slots, std::size_t count) {
    Require(slots && count, "Connectivity relation requires nonempty support");
    for (std::size_t slot = 0; slot < count; ++slot)
        Require(slots[slot] < nodes_.size(), "Connectivity relation has an absent domain node");
    auto root = Root(slots[0]);
    for (std::size_t slot = 1; slot < count; ++slot) {
        auto other = Root(slots[slot]);
        if (root == other) continue;
        if (nodes_[other].source_id < nodes_[root].source_id) std::swap(root,other);
        parent_[other] = root;
    }
}
std::size_t Components::Labels(std::vector<std::uint64_t>& labels) {
    Require(labels.size() == nodes_.size(), "Connectivity label extent changed");
    std::size_t components = 0;
    for (std::size_t node = 0; node < nodes_.size(); ++node) {
        const auto root = Root(static_cast<std::uint32_t>(node));
        labels[node] = nodes_[root].source_id;
        components += root == node;
    }
    return components;
}
void Partition(tl::util::ConstView<tl::fea::NodalDomainNode> nodes, Data& data) {
    Require(data.element_label.size() == nodes.size() && data.transfer_label.size() == nodes.size() &&
        data.node_roles.size() == nodes.size(), "Connectivity output extents differ");
    std::size_t offset = 0;
    for (const auto& row : data.relations) {
        Require(static_cast<std::size_t>(row.kind) < KindCount && row.source_id && row.slot_count &&
            row.slot_offset == offset && row.slot_count <= data.slots.size()-offset,
            "Connectivity relation layout or identity is malformed");
        constexpr std::size_t widths[]{4,3,4,8,8,6,2,2,0,0,5,1};
        const auto width = widths[static_cast<std::size_t>(row.kind)];
        Require(width ? row.slot_count == width : row.slot_count >= 2 && row.slot_count <= 1024,
                "Connectivity typed support width differs");
        Require((row.kind <= Kind::Type25 && (row.role == Role::Constitutive ||
                    (row.kind <= Kind::Qbat && row.role == Role::RigidSkin))) ||
                (row.kind >= Kind::PartRoot && row.kind <= Kind::Cin && row.role == Role::Constraint) ||
                (row.kind == Kind::PointMass && row.role == Role::CoefficientOnly),
                "Connectivity relation role does not match its typed family");
        Require((row.role == Role::RigidSkin) == (row.rigid_root_index != UINT16_MAX),
                "Connectivity rigid-skin root disposition is inconsistent");
        for (std::size_t slot = 0; slot < row.slot_count; ++slot)
            Require(data.slots[offset+slot] < nodes.size(), "Connectivity tail support is absent");
        offset += row.slot_count;
    }
    Require(offset == data.slots.size(), "Connectivity has unused support slots");
    Components components(nodes);
    Counts counts;
    counts.nodes = nodes.size(); counts.relations = data.relations.size(); counts.slots = data.slots.size();
    std::fill(data.node_roles.begin(),data.node_roles.end(),0);
    for (const auto& row : data.relations) {
        ++counts.by_kind[static_cast<std::size_t>(row.kind)];
        counts.rigid_skin_parents += row.role == Role::RigidSkin;
        const auto flag = row.role == Role::Constitutive ? ElementIncidence :
            row.role == Role::RigidSkin ? RigidSkin : row.role == Role::Constraint ? ConstraintSupport : PointMass;
        for (std::size_t slot = row.slot_offset; slot < row.slot_offset+row.slot_count; ++slot)
            data.node_roles[data.slots[slot]] |= flag;
        if (row.role == Role::Constitutive) components.Join(data.slots.data()+row.slot_offset,row.slot_count);
    }
    counts.element_components = components.Labels(data.element_label);
    for (const auto& row : data.relations)
        if (row.role == Role::Constraint) components.Join(data.slots.data()+row.slot_offset,row.slot_count);
    counts.transfer_components = components.Labels(data.transfer_label);
    for (const auto flags : data.node_roles)
        counts.mass_without_element_incidence += (flags & PointMass) && !(flags & ElementIncidence);
    data.counts = counts;
}
} // namespace detail
} // namespace crash::cases::vehicle_startup::connectivity
