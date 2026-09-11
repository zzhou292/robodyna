#include "MidlayerDeclarations.h"
#include <cmath>

namespace crash::modelio::vehicle::resolution {
NativeMappingValues MapNativeParents(const std::vector<SectionParentResolution>& parents,
    const std::vector<SectionPartResolution>& parts, const std::vector<PartDisposition>& original) {
    output::Require(parents.size() <= 524288 && parts.size() <= 1024 && parts.size() == original.size(),
                    "Native formulation mapping exceeds source scope");
    NativeMappingValues next;
    next.mapping.resize(parents.size());
    next.parents.resize(parents.size());
    for (std::size_t e = 0; e < parents.size(); ++e) {
        const auto& parent = parents[e];
        output::Require(parent.part_index < parts.size() && parent.source_parent_id &&
            (parent.topology == SourceShellTopology::Q4 || parent.topology == SourceShellTopology::T3),
            "Invalid native parent source mapping");
        const auto& part = parts[parent.part_index];
        if (part.status == SectionDisposition::Unresolved) continue;
        output::Require(part.status == SectionDisposition::Existing ||
            part.status == SectionDisposition::ConstantFailure || part.status == SectionDisposition::GlassTab1 ||
            part.status == SectionDisposition::Midlayer, "Invalid section disposition");
        const auto& source = original[parent.part_index];
        output::Require(source.part_id && source.material_id && source.section_id,
                        "Missing original native parent identity");
        auto& mapping = next.mapping[e];
        if (parent.topology == SourceShellTopology::T3) {
            mapping = {tl::fea::ShellBindingFamily::T3,next.counts.t3++};
        } else if (part.status == SectionDisposition::Midlayer) {
            mapping = {tl::fea::ShellBindingFamily::Qbat,next.counts.qbat++};
        } else {
            mapping = {tl::fea::ShellBindingFamily::Qeph,next.counts.qeph++};
        }
        auto& native = next.parents[e];
        native.source = {mapping.family,mapping.family_index,parent.source_parent_id,
                         source.part_id,source.material_id,source.section_id};
        if (part.status == SectionDisposition::ConstantFailure || part.status == SectionDisposition::Midlayer) {
            output::Require(std::isfinite(part.failure_strain) && part.failure_strain > 0,
                            "Invalid constant failure declaration");
            native.policy = tl::fea::ShellFailurePolicy::ConstantAllPoints;
            native.constant.failure_strain = part.failure_strain;
        } else if (part.status == SectionDisposition::GlassTab1) {
            output::Require(std::isfinite(part.failure_strain) && part.failure_strain > 0,
                            "Invalid glass failure declaration");
            native.policy = tl::fea::ShellFailurePolicy::Tab1AnyPoint;
            native.tab1.table = {{-.3,0,.3},part.failure_strain};
            native.tab1.parent_policy = tl::fea::sections::ShellTab1ParentPolicy::AnyPoint;
        }
    }
    return next;
}
} // namespace crash::modelio::vehicle::resolution
