#include "Internal.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::tied_shell::packing_detail {
namespace {
void DefaultFields(const std::string& card, unsigned first) {
    output::Require(card.size() <= 80, "Unsupported tied packing card width");
    for (unsigned field = first; field < 8; ++field) {
        const auto value = vehicle::detail::SourceScalar(card, field, 10);
        output::Require(!value || *value == 0,
                        "Tied packing requires default source options");
    }
}
void Set(const SourceEvidence& source) {
    const bool title = source.block.keyword == "*SET_PART_LIST_TITLE";
    output::Require(title || source.block.keyword == "*SET_PART_LIST",
                    "Tied packing requires a single additive PART clause");
    output::Require(source.cards.size() > std::size_t(title), "Missing tied set header");
    DefaultFields(source.cards[title].second, 1);
}
}
void CheckPolicy(const Data& d) {
    Set(d.sources.at(d.slave_set_source));
    Set(d.sources.at(d.master_set_source));
    const auto& contact = d.sources.at(d.contact_source);
    output::Require(contact.block.keyword == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE" &&
                    contact.cards.size() == 3, "Unsupported tied packing contact policy");
    DefaultFields(contact.cards[0].second, 4);
    DefaultFields(contact.cards[1].second, 0);
    DefaultFields(contact.cards[2].second, 0);
    output::Require(d.classification == NativeReadiness::Unresolved &&
                    d.search == NativeReadiness::Unresolved,
                    "Tied packing cannot imply classification or search admission");
}
} // namespace crash::modelio::tied_shell::packing_detail
