#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <set>

namespace crash::modelio::physical_scope::detail {
std::vector<Spotweld> ReadSpotwelds(const std::vector<tied_shell::SourceEvidence>& sources, Limits limits) {
    namespace cards = assembly::reader::auxiliary;
    std::vector<Spotweld> result;
    std::set<SourceId> ids;
    for (std::size_t block = 0; block < sources.size(); ++block) {
        const auto& source = sources[block];
        if (source.block.keyword.find("*CONSTRAINED_SPOTWELD") != 0) continue;
        Require(source.block.keyword == "*CONSTRAINED_SPOTWELD_ID" && source.cards.size() % 2 == 0,
                "Unsupported or incomplete original spotweld cards");
        for (std::size_t card = 0; card < source.cards.size(); card += 2) {
            Require(result.size() < limits.spotwelds, "Original spotweld count exceeds cap");
            const auto& first = source.cards[card].second;
            const auto& second = source.cards[card + 1].second;
            Require(cards::BlankTail(first, 80) && cards::BlankTail(second, 80),
                    "Original spotweld card exceeds eight fixed-width fields");
            Spotweld row;
            row.id = cards::Id(first, 0, 10);
            row.nodes = {cards::Id(second, 0, 10), cards::Id(second, 10, 10)};
            Require(ids.insert(row.id).second && row.nodes[0] != row.nodes[1],
                    "Duplicate spotweld identity or repeated endpoint");
            row.source_index = block;
            row.first_card = card;
            // Same literal-only subset as the existing component reader. All
            // nonblank optional fields remain in the retained raw source cards.
            row.default_only = cards::BlankTail(first, 10) && cards::BlankTail(second, 20);
            result.push_back(row);
        }
    }
    return result;
}
} // namespace crash::modelio::physical_scope::detail
