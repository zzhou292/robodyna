#include "Fields.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <set>

namespace crash::modelio::vehicle::rigid_part::point_mass::detail {
std::vector<Record> Read(const std::vector<tied_shell::SourceEvidence>& sources,
    double mass_to_kg,Limits limits) {
    using output::Require;
    namespace fields=assembly::reader::auxiliary;
    const Limits hard;
    Require(limits.records && limits.records<=hard.records && limits.blocks &&
        limits.blocks<=hard.blocks && limits.host_bytes && limits.host_bytes<=hard.host_bytes,
        "Invalid rigid point-mass source caps");
    Require(sources.size()<=limits.blocks && std::isfinite(mass_to_kg) && mass_to_kg>0,
            "Rigid point-mass source count or units are invalid");
    std::size_t cards=0;
    for(const auto& source:sources) if(source.block.keyword=="*ELEMENT_MASS") {
        Require(source.cards.size()<=limits.records-cards,"Rigid point-mass card count exceeds cap");
        cards+=source.cards.size();
    }
    Require(cards<=(limits.host_bytes/ (sizeof(Record)+128)),"Rigid point-mass field scratch exceeds cap");
    std::vector<Record> records;
    records.reserve(cards);
    std::set<std::uint64_t> identities;
    std::size_t previous_block=0;
    for(std::size_t s=0;s<sources.size();++s) {
        const auto& source=sources[s];
        if(source.block.keyword!="*ELEMENT_MASS") continue;
        Require(source.block.filename=="yaris-coarse-v1l.key" &&
            source.block.first_line>previous_block && source.block.last_line>=source.block.first_line,
            "Rigid point-mass block source association changed");
        previous_block=source.block.last_line;
        std::size_t previous_card=source.block.first_line;
        for(std::size_t c=0;c<source.cards.size();++c) {
            const auto& card=source.cards[c];
            Require(card.first>previous_card && card.first<=source.block.last_line,
                    "Rigid point-mass card source association changed");
            previous_card=card.first;
            if(card.second.empty()) continue;
            assembly::AuxiliaryPointMass value;
            value.source_element_id=fields::Id(card.second,0,8);
            value.source_node_id=fields::Id(card.second,8,8);
            value.supplied_mass_source=fields::Number(card.second,16,16);
            value.supplied_mass_kg=value.supplied_mass_source*mass_to_kg;
            Require(identities.insert(value.source_element_id).second && fields::BlankTail(card.second,32) &&
                value.supplied_mass_source>0 && std::isfinite(value.supplied_mass_kg) && value.supplied_mass_kg>0,
                "Invalid or duplicate original additive point-mass record");
            value.source_block_line=source.block.first_line;
            value.source_card_index=c;
            records.push_back({value,s});
        }
    }
    return records;
}
} // namespace crash::modelio::vehicle::rigid_part::point_mass::detail
