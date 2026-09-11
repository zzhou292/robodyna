#include "Internal.h"
#include "lib_src/elements/solid18/Solid18Reference.h"
#include "lib_src/elements/solid24/Solid24Reference.h"
#include "lib_src/elements/solid6z/Solid6zReference.h"
#include "lib_src/elements/solid18/law44/Reference.h"
#include <algorithm>

namespace crash::modelio::solid_source::detail {
namespace {
template<class Input> void Pack(Input& input, const Row& row, const Part& part,
                                const std::vector<double>& positions, unsigned count) {
    input.source_element_id = row.element_id;
    input.source_part_id = row.part_id;
    input.source_section_id = part.section_id;
    input.source_material_id = part.material_id;
    input.density_kg_m3 = part.density_kg_m3;
    for (unsigned slot = 0; slot < count; ++slot) {
        const auto raw = count == 6 ? row.six_to_raw[slot] : slot;
        const auto node = row.canonical_nodes[raw];
        Require(node < positions.size() / 3, "Selected solid native coordinate index changed");
        input.source_node_id[slot] = row.raw_node_ids[raw];
        input.position_m[slot] = {positions[3 * node], positions[3 * node + 1], positions[3 * node + 2]};
    }
}
template<class Reference, class Input>
void Append(std::vector<Reference>& references, Input& input, Row& row) {
    Reference reference;
    const auto status = InitializeReference(input, reference);
    if (static_cast<unsigned>(status) != 0)
        throw std::runtime_error("Native solid reference rejected original EID " +
            std::to_string(row.element_id) + " (status " + std::to_string(static_cast<unsigned>(status)) + ")");
    row.reference_index = references.size();
    references.push_back(reference);
}
}
void PrepareReferences(const source::CanonicalData& source, Data& data, Limits) {
    const auto positions = Decode<double>(source, "node_positions");
    Require(positions.size() == 3 * source.canonical_nodes, "Solid source coordinate extent changed");
    const auto count = [&](Family family) {
        return std::count_if(data.rows.begin(), data.rows.end(), [&](const Row& row) { return row.family == family; });
    };
    data.solid18.reserve(count(Family::Solid18));
    data.solid24.reserve(count(Family::Solid24));
    data.solid6z.reserve(count(Family::Solid6z));
    data.solid18_law44.reserve(count(Family::Solid18Law44));
    for (auto& row : data.rows) {
        Require(row.part_index < data.parts.size(), "Solid row material association is invalid");
        const auto& part = data.parts[row.part_index];
        Require((row.family == Family::Solid18) == (part.material_law == MaterialLaw::Law36),
                "Selected solid family/material law association changed");
        Require((row.family == Family::Solid18Law44) == (part.material_law == MaterialLaw::Law44),
                "Selected rear solid family/material association changed");
        if (row.family == Family::Solid18Law44) {
            tl::fea::solid18::law44::ReferenceInput input;
            Pack(input, row, part, positions, 8);
            input.profile = tl::fea::solid18::law44::Profile();
            Append(data.solid18_law44, input, row);
        } else if (row.family == Family::Solid18) {
            tl::fea::solid18::ReferenceInput input;
            Pack(input, row, part, positions, 8);
            Append(data.solid18, input, row);
        } else if (row.family == Family::Solid24) {
            tl::fea::solid24::ReferenceInput input;
            Pack(input, row, part, positions, 8);
            input.profile.reference_strain = tl::fea::solid24::ReferenceStrain::TotalLagrangian10;
            input.profile.working_length = tl::fea::solid24::WorkingLengthUnit::Millimetre;
            Append(data.solid24, input, row);
        } else {
            tl::fea::solid6z::ReferenceInput input;
            Pack(input, row, part, positions, 6);
            Append(data.solid6z, input, row);
        }
    }
}
} // namespace crash::modelio::solid_source::detail
