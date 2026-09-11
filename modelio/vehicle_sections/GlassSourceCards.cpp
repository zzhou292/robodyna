#include "GlassDeclarations.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include <algorithm>
#include <sstream>

namespace crash::modelio::vehicle::resolution {
using namespace assembly::reader;
namespace {
std::vector<assembly::DeclarationCard> OriginalCards(const assembly::SourceBlock& source) {
    std::vector<assembly::DeclarationCard> cards;
    std::istringstream stream(source.raw_text);
    std::string line;
    std::size_t number = source.first_line;
    while (std::getline(stream, line)) {
        const auto source_line = number++;
        const auto first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && (line[first] == '*' || line[first] == '$')) continue;
        const auto comment = line.find('$');
        if (comment != std::string::npos) line.resize(comment);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        assembly::DeclarationCard card;
        card.source_line = source_line;
        for (unsigned field = 0; field < 8; ++field) {
            const auto value = detail::SourceScalar(line, field);
            card.values.push_back(value);
            if (!value) card.blank_mask |= 1u << field;
        }
        cards.push_back(std::move(card));
        Require(cards.size() <= 4, "Too many original glass data cards");
    }
    return cards;
}
void Shape(const std::vector<assembly::DeclarationCard>& cards, std::initializer_list<unsigned> masks) {
    Require(cards.size() == masks.size(), "Glass source card count changed");
    std::size_t i = 0;
    for (const auto mask : masks) {
        Require(cards[i].values.size() == 8 && cards[i].blank_mask == mask,
                "Unsupported original glass card options");
        ++i;
    }
}
double At(const assembly::DeclarationCard& card, unsigned field) {
    return RequiredMaterialCard(card, field);
}
}
void CheckGlassCardOrder(const assembly::SourceBlock& block,
                         const std::vector<assembly::DeclarationCard>& cards) {
    const auto original = OriginalCards(block);
    Require(cards.size() == original.size(), "Glass source card coverage changed");
    for (std::size_t i = 0; i < cards.size(); ++i) {
        Require(cards[i].source_line == original[i].source_line,
                "Glass source card order/location changed");
    }
}
void CheckGlassMaterialCards(const std::vector<assembly::DeclarationCard>& cards) {
    Shape(cards, {128, 115, 255, 255});
    const auto& first = cards[0];
    const auto& second = cards[1];
    Require(At(first,0) > 0 && At(first,1) > 0 && At(first,2) > 0 &&
            At(first,3) >= 0 && At(first,3) < .5 && At(first,4) > 0 &&
            At(first,5) >= 0 && At(first,5) < At(first,2) && At(first,6) > 0 &&
            At(second,2) == 0 && At(second,3) == 0 && At(second,7) == 1,
            "Glass requires analytic MAT123 with explicit LCSS0/LCSR0/NUMINT1");
}
void CheckGlassSectionCards(const std::vector<assembly::DeclarationCard>& cards) {
    Require(cards.size() == 2, "Glass requires two section cards");
    const auto mask = cards[1].blank_mask;
    Require(mask == 240 || mask == 224, "Unsupported glass section placement fields");
    Shape(cards, {244, mask});
    Require(At(cards[0],0) > 0 && At(cards[0],1) == 2 && At(cards[0],3) == 3,
            "Glass requires original ELFORM2/NIP3");
    for (unsigned i = 0; i < 4; ++i) {
        Require(At(cards[1],i) > 0 && At(cards[1],i) == At(cards[1],0),
                "Glass requires uniform positive thickness");
    }
    if (const auto nloc = cards[1].values[4]) {
        Require(*nloc == -1 || *nloc == 0 || *nloc == 1, "Unsupported original glass NLOC");
    }
}
bool EligibleGlass(const PartDisposition& part) {
    if (part.status != Disposition::Unresolved || part.obligations.empty() ||
        std::any_of(part.obligations.begin(), part.obligations.end(), [](const auto& row) {
            return row.stage != "material" && row.stage != "section";
        })) return false;
    const auto& material = part.unresolved_sources[2];
    if (material.keyword != "*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY" && material.keyword != "*MAT_123") return false;
    if (part.unresolved_sources[1].keyword != "*SECTION_SHELL") return false;
    try {
        CheckGlassMaterialCards(OriginalCards(material));
        CheckGlassSectionCards(OriginalCards(part.unresolved_sources[1]));
        return true;
    } catch (const std::runtime_error&) {
        return false;
    }
}
} // namespace crash::modelio::vehicle::resolution
