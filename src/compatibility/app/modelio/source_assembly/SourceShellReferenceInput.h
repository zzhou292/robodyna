#pragma once
#include "SourceAssemblyData.h"
#include "output/ArtifactIO.h"
#include <limits>
#include <type_traits>

namespace crash::modelio::assembly {
struct SourceReferenceNode { std::uint64_t source_id=0; tl::math::Vec3 position_m{}; };
// Shared source-to-native packing only. Coordinates and coefficients are copied
// without projection, normalization, mass construction or mechanics admission.
template<class Input, std::size_t Arity>
Input PackShellReference(const SourceReferenceNode (&nodes)[Arity],
                         const Material& material,const Section& section) {
    Input input;
    static_assert(std::extent_v<decltype(input.position)> == Arity);
    using NodeId=std::remove_reference_t<decltype(input.node_ids[0])>;
    for(std::size_t n=0;n<Arity;++n) {
        output::Require(nodes[n].source_id<=std::numeric_limits<NodeId>::max(),
                        "Native family cannot represent source NID");
        input.position[n]=nodes[n].position_m;
        input.node_ids[n]=static_cast<NodeId>(nodes[n].source_id);
    }
    input.density=material.density_kg_m3;
    input.young_modulus=material.young_pa;
    input.poisson_ratio=material.poisson_ratio;
    input.thickness=section.thickness_m[0];
    return input;
}
} // namespace crash::modelio::assembly
