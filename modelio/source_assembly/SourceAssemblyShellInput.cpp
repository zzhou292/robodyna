#include "SourceAssemblyShellInput.h"
#include "output/ArtifactIO.h"
#include <limits>
#include <type_traits>

namespace crash::modelio::assembly {
namespace {
template<class Binding, unsigned Arity>
Binding MakeBinding(const Data& source, const Parent& parent) {
    output::Require(parent.arity == Arity, "Assembly shell family arity mismatch");
    Binding binding; binding.source_parent_id = parent.source_id;
    using NodeId = std::remove_reference_t<decltype(binding.reference.node_ids[0])>;
    for (unsigned n = 0; n < Arity; ++n) {
        const auto index = parent.nodes[n]; const auto& node = source.nodes.at(index);
        output::Require(node.source_id <= std::numeric_limits<NodeId>::max(), "Native family cannot represent source NID");
        binding.nodes[n] = index;
        binding.reference.position[n] = node.position_m;
        binding.reference.node_ids[n] = static_cast<NodeId>(node.source_id);
    }
    const auto& material = source.materials.at(parent.material_index);
    const auto& section = source.sections.at(parent.section_index);
    binding.reference.density = material.density_kg_m3;
    binding.reference.young_modulus = material.young_pa;
    binding.reference.poisson_ratio = material.poisson_ratio;
    binding.reference.thickness = section.thickness_m[0];
    return binding;
}
}  // namespace
SourceAssemblyShellInput::SourceAssemblyShellInput(const SourceAssembly& source) : source_(source) {
    const auto& data = source_.data(); node_count_ = data.nodes.size();
    qeph_.reserve(data.qeph_count); t3_.reserve(data.t3_count);
    qeph_parents_.reserve(data.qeph_count); t3_parents_.reserve(data.t3_count);
    for (const auto& parent : data.parents) {
        if (parent.family == ShellFamily::Qeph) {
            output::Require(parent.family_index == qeph_.size(), "QEPH input family order changed");
            qeph_.push_back(MakeBinding<tl::fea::ShellQephBindingInput, 4>(data, parent));
            qeph_parents_.push_back(parent.index);
        } else {
            output::Require(parent.family_index == t3_.size(), "T3 input family order changed");
            t3_.push_back(MakeBinding<tl::fea::ShellT3BindingInput, 3>(data, parent));
            t3_parents_.push_back(parent.index);
        }
    }
}
tl::fea::ShellBatchCollectionInput SourceAssemblyShellInput::input() const noexcept {
    return {qeph_.empty() ? nullptr : qeph_.data(), t3_.empty() ? nullptr : t3_.data(), qeph_.size(), t3_.size(), node_count_};
}
}  // namespace crash::modelio::assembly
