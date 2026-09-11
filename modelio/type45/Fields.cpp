#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <set>

namespace crash::modelio::type45::detail {
bool IsJoint(const std::string& keyword) { return keyword.rfind("*CONSTRAINED_JOINT_", 0) == 0; }
std::vector<Row> Read(const std::vector<Evidence>& sources, Limits limits) {
    std::vector<Row> rows;
    std::set<std::uint64_t> ids;
    rows.reserve(limits.rows);
    for (std::size_t index = 0; index < sources.size(); ++index) {
        const auto& source = sources[index];
        if (!IsJoint(source.block.keyword)) continue;
        unsigned kind = 0, count = 0;
        if (source.block.keyword == "*CONSTRAINED_JOINT_SPHERICAL_ID") { kind = 1; count = 2; }
        if (source.block.keyword == "*CONSTRAINED_JOINT_REVOLUTE_ID") { kind = 2; count = 4; }
        if (source.block.keyword == "*CONSTRAINED_JOINT_CYLINDRICAL_ID") { kind = 3; count = 4; }
        Require(kind && rows.size() < limits.rows && source.cards.size() == 2,
                "Unsupported or oversized original joint declaration");
        Require(source.block.filename == "yaris-coarse-v1l.key" &&
            output::Sha256(source.block.raw_text) == source.block.sha256,
            "Original joint raw evidence changed");
        const auto cards = vehicle::detail::ReadSourceCards(source.block, 1, 1);
        Require(cards.size() == 1 && cards[0].source_line == source.cards[1].first &&
            cards[0].raw_text == source.cards[1].second,
            "Original joint card association changed");
        const auto& card = cards[0];
        Row row;
        row.source_id = assembly::reader::auxiliary::Id(source.cards[0].second, 0, 10);
        Require(ids.insert(row.source_id).second, "Repeated original joint identity");
        row.source_index = index; row.header_line = source.cards[0].first; row.card_line = card.source_line;
        row.property_index = kind - 1; row.source_node_count = count; row.blank_mask = card.blank_mask;
        Require(assembly::reader::auxiliary::BlankTail(card.raw_text, 80), "Original joint card has extra fields");
        for (unsigned field = 0; field < 8; ++field) {
            const auto value = card.values[field];
            if (kind == 3 && (field == 4 || field == 5)) {
                row.unused_columns[field - 4] = value;
                continue;
            }
            if (field >= count) {
                Require(!value, "Original joint optional field is not literal blank");
                continue;
            }
            Require(value && *value > 0 && *value <= UINT32_MAX && *value == std::floor(*value),
                    "Invalid original joint node identity");
            row.nodes[field].source_id = static_cast<std::uint64_t>(*value);
            row.nodes[field].use = field < 2 ? NodeUse::Endpoint : field == 2 ? NodeUse::InitialAxis : NodeUse::OriginalEvidence;
        }
        const unsigned consumed = kind == 1 ? 2 : 3;
        for (unsigned n = 0; n < consumed; ++n)
            for (unsigned previous = 0; previous < n; ++previous)
                Require(row.nodes[n].source_id != row.nodes[previous].source_id,
                        "Repeated consumed original joint node");
        rows.push_back(row);
    }
    return rows;
}
void ResolveProperties(Data& data) {
    for (unsigned kind = 0; kind < 3; ++kind) {
        auto& property = data.properties[kind];
        property.value.kind = static_cast<native::Kind>(kind + 1);
        property.value.working_units = native::WorkingUnits::MillimetreTonneSecond;
        property.value.automatic_stiffness_scale = .01;
        property.value.critical_damping_ratio = .05;
        property.value.free_stiffness = {};
        property.value.free_viscosity = {};
        const auto first = std::find_if(data.rows.begin(), data.rows.end(),
            [&](const Row& row) { return row.property_index == kind; });
        Require(first != data.rows.end(), "Missing original joint kind");
        property.origin_joint_id = first->source_id;
    }
}
void CheckOriginal(Data& data) {
    static constexpr std::uint64_t omitted[]{2200514, 2200515, 2200526, 2200527, 2200528, 2200529};
    Require(data.rows.size() == 44, "Original joint census changed");
    std::size_t kinds[3]{};
    for (std::size_t i = 0; i < data.rows.size(); ++i) {
        auto& row = data.rows[i];
        Require(row.source_id == 2200512 + i && row.property_index < 3, "Original joint source order changed");
        ++kinds[row.property_index];
        const bool boundary = std::binary_search(std::begin(omitted), std::end(omitted), row.source_id);
        const unsigned missing = (row.nodes[0].domain_index == SIZE_MAX) + (row.nodes[1].domain_index == SIZE_MAX);
        Require(missing == unsigned(boundary), "Original joint retained/boundary disposition changed");
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint)
            Require(row.nodes[endpoint].body.kind != BodyKind::None,
                    "Original joint endpoint source membership is unresolved");
        if (!boundary && row.property_index != 0)
            Require(row.nodes[2].domain_index != SIZE_MAX, "Required joint axis node is outside retained domain");
        row.disposition = boundary ? Disposition::OmittedAssemblyBoundary : Disposition::Required;
        data.boundaries += boundary; data.required += !boundary;
    }
    Require(data.required == 38 && data.boundaries == 6 && kinds[0] == 17 && kinds[1] == 22 && kinds[2] == 5,
            "Original joint kind/disposition census changed");
}
} // namespace crash::modelio::type45::detail
