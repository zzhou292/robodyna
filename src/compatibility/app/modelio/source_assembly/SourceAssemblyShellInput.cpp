#include "SourceAssemblyShellInput.h"
#include "SourceShellReferenceInput.h"

namespace crash::modelio::assembly {
namespace {
template<class Binding, unsigned Arity>
Binding MakeBinding(const Data& source, const Parent& parent) {
    output::Require(parent.arity == Arity, "Assembly shell family arity mismatch");
    Binding binding; binding.source_parent_id = parent.source_id;
    SourceReferenceNode nodes[Arity];
    using NodeId=std::remove_reference_t<decltype(binding.reference.node_ids[0])>;
    for (unsigned n = 0; n < Arity; ++n) {
        const auto index = parent.nodes[n]; const auto& node = source.nodes.at(index);
        // Keep the legacy per-node rejection order before reading later nodes.
        output::Require(node.source_id<=std::numeric_limits<NodeId>::max(),"Native family cannot represent source NID");
        binding.nodes[n] = index;
        nodes[n]={node.source_id,node.position_m};
    }
    const auto& material = source.materials.at(parent.material_index);
    const auto& section = source.sections.at(parent.section_index);
    binding.reference=PackShellReference<decltype(binding.reference)>(nodes,material,section);
    return binding;
}
}  // namespace
SourceAssemblyShellInput::SourceAssemblyShellInput(const SourceAssembly& source)
    :SourceAssemblyShellInput(source,QephMetricProfile::LegacyOneMetre) {}
SourceAssemblyShellInput::SourceAssemblyShellInput(const SourceAssembly& source,QephMetricProfile profile)
    :source_(source),metric_(QephReferenceMetric::Resolve(profile,source.data().units)) {
    const auto& data = source_.data(); node_count_ = data.nodes.size();
    qeph_.reserve(data.qeph_count); t3_.reserve(data.t3_count);
    qeph_parents_.reserve(data.qeph_count); t3_parents_.reserve(data.t3_count);
    for (const auto& parent : data.parents) {
        if (parent.family == ShellFamily::Qeph) {
            output::Require(parent.family_index == qeph_.size(), "QEPH input family order changed");
            auto binding=MakeBinding<tl::fea::ShellQephBindingInput,4>(data,parent);
            binding.reference=WithQephMetric(binding.reference,metric_);
            qeph_.push_back(binding);
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
