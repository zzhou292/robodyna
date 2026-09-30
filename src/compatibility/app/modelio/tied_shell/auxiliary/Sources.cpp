#include "Internal.h"

namespace crash::modelio::tied_shell::auxiliary_detail {
std::vector<SourceEvidence> Sources(const Value& file, const std::string& member, AuxiliaryLimits limits) {
    const auto& blocks = Array(file, "blocks", limits.blocks, 1);
    detail::Requests requests;
    std::size_t previous = 0, groups = 0;
    for (const auto& value : blocks.GetArray()) {
        const auto first = Unsigned(value, "first_line"), last = Unsigned(value, "last_line");
        const auto keyword = Text(value, "keyword");
        Require(Text(value, "file") == MemberName && first > previous && last >= first,
                "Auxiliary source block location/order changed");
        previous = last;
        const auto hash = Text(value, "source_block_sha256");
        output::arrays::CheckHash(hash);
        const bool group = keyword == "*CONSTRAINED_NODAL_RIGID_BODY" ||
                           keyword == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE";
        if (group) ++groups;
        if (!group && keyword.rfind("*SET_NODE_", 0) != 0 && keyword != "*NODE" &&
            keyword != "*ELEMENT_SOLID") continue;
        detail::Request request;
        request.evidence.block = {MemberName, keyword, {}, hash, first, last};
        Require(requests.emplace(first, std::move(request)).second, "Duplicate auxiliary block request");
    }
    Require(groups && groups <= limits.groups, "Auxiliary group count exceeds capacity");
    const auto& counts = Member(file, "keyword_counts");
    const auto count = [&](const char* name) {
        return counts.HasMember(name) ? Unsigned(counts, name, limits.groups) : 0;
    };
    Require(counts.IsObject() && groups == count("*CONSTRAINED_NODAL_RIGID_BODY") +
            count("*CONSTRAINED_NODAL_RIGID_BODY_TITLE"), "Auxiliary group inventory omitted a block");
    Limits source_limits;
    source_limits.member_bytes = limits.member_bytes;
    source_limits.metadata_bytes = limits.metadata_bytes;
    source_limits.blocks = limits.blocks;
    return detail::ReadRequestedSources(requests, member, source_limits);
}
} // namespace crash::modelio::tied_shell::auxiliary_detail
