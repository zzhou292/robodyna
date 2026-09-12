#pragma once
#include "SourceNodes.h"

namespace crash::cases::vehicle_dynamics::limiter {
// Streams original prepared source incidence. These are possible contributors
// to the selected M/J/STI/STIR, not per-element shares of current stiffness.
template<class Emit> void VisitRows(const vehicle_wall::VehicleWallSetup& setup,
                                  const SourceNodes& selected, Emit emit) {
    const auto& physical = setup.execution().model();
    const auto& ledger = physical.coefficients();
    const auto& shells = physical.shell_source().shells();
    const auto& refs = physical.shell_source().references();
    const auto hits = [&](const auto& slots) {
        bool hit = false;
        for(const auto slot : slots) hit |= selected.Contains(ledger.shells()->owner_index(slot));
        return hit;
    };
    for(const auto& row : refs.rows()) {
        const auto i = row.reference_index;
        using Family = vehicle_startup::ReferenceFamily;
        if(row.family == Family::Qeph && hits(shells.qeph_nodes(i))) {
            const auto& input = shells.qeph_reference(i).input;
            emit("qeph",row.element_id,row.part_id,row.section_id,row.material_id,
                 input.thickness,input.density,input.young_modulus);
        } else if(row.family == Family::T3 && hits(shells.t3_nodes(i))) {
            const auto& input = shells.t3_reference(i).input;
            emit("t3",row.element_id,row.part_id,row.section_id,row.material_id,
                 input.thickness,input.density,input.young_modulus);
        } else if(row.family == Family::Qbat && hits(shells.qbat_nodes(i))) {
            const auto& input = shells.qbat_reference(i).input().quadrilateral;
            emit("qbat",row.element_id,row.part_id,row.section_id,row.material_id,
                 input.thickness,input.density,input.young_modulus);
        }
    }
    if(const auto* solids = ledger.solids()) {
        const char* names[]{"solid18","heph","s6z","solid18_law44","solid18_law90"};
        for(const auto& parent : solids->parents()) {
            bool hit = false;
            for(unsigned slot = 0; slot < parent.node_count; ++slot) hit |= selected.Contains(parent.domain_node[slot]);
            if(hit) emit(names[static_cast<unsigned>(parent.family)],parent.source_element_id,
                parent.source_part_id,parent.source_section_id,parent.source_material_id,0,0,0);
        }
    }
    if(const auto* beams = physical.structural_beams()) {
        for(const auto& parent : beams->parents()) {
            if(!selected.Contains(parent.domain_nodes[0]) && !selected.Contains(parent.domain_nodes[1])) continue;
            const auto& input = parent.reference.input();
            emit("beam18",input.source_element_id,input.source_part_id,input.source_section_id,
                 input.source_material_id,0,0,0);
        }
    }
    if(const auto* type13 = ledger.type13()) {
        const auto rows = type13->records();
        for(std::size_t i = 0; i < rows.size(); i += 2) {
            if(!selected.Contains(rows[i].value.global_node) && !selected.Contains(rows[i+1].value.global_node)) continue;
            emit("type13",rows[i].value.source_element_id,0,rows[i].value.source_property_id,0,0,0,0);
        }
    }
    if(const auto* type25 = ledger.type25()) {
        for(std::size_t i = 0; i < type25->connection_count(); ++i) {
            const auto& row = type25->connections()[i];
            if(selected.Contains(row.global_node[0]) || selected.Contains(row.global_node[1]))
                emit("type25",row.source_element_id,0,0,0,0,0,0);
        }
    }
    if(const auto* masses = ledger.element_mass()) {
        for(const auto& row : masses->records()) if(selected.Contains(row.source.domain_node))
            emit("element_mass",row.source.source_element_id,0,0,0,0,0,0);
    }
}
} // namespace crash::cases::vehicle_dynamics::limiter
