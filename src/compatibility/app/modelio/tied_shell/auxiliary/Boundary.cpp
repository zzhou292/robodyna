#include "Internal.h"

namespace crash::modelio::tied_shell::auxiliary_detail {
void Boundary(Draft& draft, const Data& declaration, const Value& files,
              OriginalWallPolicy policy, AuxiliaryLimits limits) {
    auto& d = draft.data;
    d.wall_policy = policy;
    std::set<std::pair<std::string, std::size_t>> retained_groups;
    for (const auto& group : d.groups) {
        const auto& source = d.sources[group.evidence.source].block;
        retained_groups.emplace(source.filename, source.first_line);
    }
    for (const auto& block : declaration.unresolved_constraints) {
        if (block.keyword.rfind("*RIGIDWALL_", 0) == 0) continue;
        if (retained_groups.count({block.filename, block.first_line})) continue;
        d.other_unresolved_constraints.push_back(block);
    }
    std::size_t total_blocks = 0;
    for (auto file = files.MemberBegin(); file != files.MemberEnd(); ++file) {
        const std::string name(file->name.GetString(), file->name.GetStringLength());
        const auto& blocks = Array(file->value, "blocks", limits.blocks);
        Require(blocks.Size() <= limits.blocks-total_blocks, "Auxiliary boundary block cap exceeded");
        total_blocks += blocks.Size();
        const auto& counts = Member(file->value, "keyword_counts");
        Require(counts.IsObject(), "Missing original boundary keyword census");
        std::map<std::string, std::size_t> actual;
        std::size_t previous = 0;
        for (const auto& row : blocks.GetArray()) {
            const auto first = Unsigned(row, "first_line"), last = Unsigned(row, "last_line");
            Require(Text(row, "file") == name && first > previous && last >= first,
                    "Original boundary block identity/order changed");
            previous = last;
            const auto keyword = Text(row, "keyword");
            if (keyword.rfind("*RIGIDWALL_", 0) != 0) continue;
            UnresolvedBlock block{name, keyword, Text(row, "source_block_sha256"), first, last};
            output::arrays::CheckHash(block.sha256);
            ++actual[keyword];
            d.original_walls.push_back(std::move(block));
        }
        for (auto count = counts.MemberBegin(); count != counts.MemberEnd(); ++count) {
            const std::string keyword(count->name.GetString(), count->name.GetStringLength());
            if (keyword.rfind("*RIGIDWALL_", 0) == 0)
                Require(Unsigned(count->value, limits.blocks) == actual[keyword],
                        "Original wall policy inventory omitted a block");
        }
        for (const auto& [keyword, count] : actual)
            Require(counts.HasMember(keyword.c_str()) && Unsigned(counts, keyword.c_str()) == count,
                    "Original wall policy census omitted a keyword");
        if (!actual.empty()) {
            const auto hash = Text(file->value, "sha256");
            output::arrays::CheckHash(hash);
            d.wall_source_files.push_back({name, hash});
        }
    }
    // A zero wall population may be authentic in a tiny fixture. Replacement
    // explicitly applies to the entire inventoried population, never a subset.
}
} // namespace crash::modelio::tied_shell::auxiliary_detail
