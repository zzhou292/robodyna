#pragma once
#include "ShellCollectionContactGeometry.h"
#include <optional>
#include <vector>

namespace crash::cases {
struct ShellCollectionContactGeometry::Impl {
    Impl(const tl::fea::ShellBatchBinding& source,const tl::fea::ShellNodeMap* map)
        : binding(source),mapping(map?std::optional<tl::fea::ShellNodeMap>(*map):std::nullopt) {}
    tl::fea::ShellBatchBinding binding;
    std::optional<tl::fea::ShellNodeMap> mapping;
    std::vector<double> coordinates;
    std::vector<ShellContactParent> parents;
    tlfea::contact::NodalWallWeights weights;
    std::array<tlfea::contact::Vec3,2> bounds{};
    std::size_t startup_bytes=0;
    std::size_t node_count() const noexcept {
        return mapping?mapping->owner_node_count():binding.node_count();
    }
    std::size_t node(std::size_t local) const noexcept {
        return mapping?mapping->owner_index(local):local;
    }
    void PrepareCoordinates();
    ShellContactGeometryReport PrepareReferences(const ShellContactGeometryLimits&);
};
} // namespace crash::cases
