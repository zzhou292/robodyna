#include "SourceCards.h"
#include <sstream>

namespace crash::modelio::vehicle::detail {
std::vector<assembly::DeclarationCard> ReadSourceCards(const assembly::SourceBlock& source,
    std::size_t skip_data_cards, std::size_t maximum_cards) {
    output::Require(skip_data_cards <= 1 && maximum_cards && maximum_cards <= 4 &&
                    source.first_line && source.last_line >= source.first_line &&
                    source.last_line < SIZE_MAX,
                    "Invalid bounded source card extent");
    std::vector<assembly::DeclarationCard> cards;
    std::istringstream stream(source.raw_text);
    std::string line;
    std::size_t number = source.first_line, skipped = 0;
    while (std::getline(stream, line)) {
        const auto source_line = number++;
        const auto first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && (line[first] == '*' || line[first] == '$')) continue;
        if (skipped < skip_data_cards) { ++skipped; continue; }
        output::Require(cards.size() < maximum_cards, "Too many original numerical cards");
        const auto comment = line.find('$');
        if (comment != std::string::npos) line.resize(comment);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto last = line.find_last_not_of(" \t");
        line.resize(last == std::string::npos ? 0 : last + 1);
        assembly::DeclarationCard card;
        card.source_line = source_line;
        card.raw_text = line;
        for (unsigned field = 0; field < 8; ++field) {
            const auto value = SourceScalar(line, field);
            card.values.push_back(value);
            if (!value) card.blank_mask |= 1u << field;
        }
        cards.push_back(std::move(card));
    }
    output::Require(number == source.last_line + 1 && skipped == skip_data_cards,
                    "Original source card coverage changed");
    return cards;
}
} // namespace crash::modelio::vehicle::detail
