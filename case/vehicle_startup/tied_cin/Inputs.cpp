#include "Internal.h"
#include "lib_src/assembly/NodalDomainIdentity.h"

namespace crash::cases::vehicle_startup::tied_cin_detail {
namespace {
template<class T> std::vector<T> Decode(const tied::source::CanonicalData& canonical,const char* name) {
    const auto& array = tied::source::FindArray(canonical,name);
    return output::arrays::Decode<T>(array.descriptor,array.bytes);
}
struct SourceNodes {
    std::vector<tied::SourceId> ids;
    std::vector<double> positions;
    native_search::Vec3 Position(const tied::SearchGeometryData& geometry,std::size_t working) const {
        output::Require(working < geometry.canonical_nodes.size() && working < geometry.working_positions.size(),
                        "CIN working-node association differs");
        const auto row = geometry.canonical_nodes[working];
        output::Require(row < ids.size(),"CIN canonical node index is invalid");
        const auto& raw = geometry.working_positions[working];
        const auto scale = geometry.working_length_to_m;
        const native_search::Vec3 source{positions[3*row],positions[3*row+1],positions[3*row+2]};
        const native_search::Vec3 working_si{raw[0]*scale,raw[1]*scale,raw[2]*scale};
        output::Require(tl::fea::nodal_domain_detail::SamePosition(source,working_si),
                        "CIN original working-to-SI coordinate bits changed");
        return source;
    }
    tied::SourceId Id(const tied::SearchGeometryData& geometry,std::size_t working) const {
        output::Require(working < geometry.canonical_nodes.size() && geometry.canonical_nodes[working] < ids.size(),
                        "CIN canonical NID association is invalid");
        return ids[geometry.canonical_nodes[working]];
    }
};
}
Inputs Pack(const tied::source::CanonicalData& canonical,const tied::Data& declaration,
        const tied::PackingData& packing,const tied::SearchGeometryData& geometry,
        const native_search::FinalizationMaps& finalized,const TiedClassificationData& classified,
        const native_search::PostKinChkResult& post) {
    using output::Require;
    const auto count = finalized.slaves.size();
    Require(count && count == post.slaves().count && count == classified.slaves.size() &&
            count == finalized.selected_masters.size() && count == finalized.st.size() &&
            classified.cin_count == count && classified.penalty_count == 0 &&
            packing.master_rows.size() == geometry.masters.size() &&
            geometry.secondary_working_nodes.size() == declaration.slave_nodes.size(),
            "CIN requires one complete finalized CIN source scope");
    SourceNodes source{Decode<tied::SourceId>(canonical,"node_ids"),Decode<double>(canonical,"node_positions")};
    Require(source.ids.size() == canonical.canonical_nodes && source.positions.size() == 3*source.ids.size(),
            "CIN canonical node arrays changed extent");
    Inputs out;
    out.reserve(count);
    for (std::size_t row = 0; row < count; ++row) {
        const auto original = finalized.slaves[row];
        const auto rank = finalized.selected_masters[row];
        Require(original < declaration.slave_nodes.size() && rank && rank <= geometry.masters.size() &&
                classified.slaves[row].original_nsv_row == original &&
                classified.slaves[row].source_node_id == declaration.slave_nodes[original].id &&
                post.slaves().data[row].before.source_id == declaration.slave_nodes[original].id,
                "CIN finalized original NSV/IRECT identity differs");
        const auto& master = geometry.masters[rank-1];
        Require(master.declaration_row == packing.master_rows[rank-1] && master.declaration_row < declaration.masters.size(),
                "CIN original declared master association differs");
        const auto& element = declaration.masters[master.declaration_row];
        Require(element.family == tied::ElementFamily::Shell && element.part_index < declaration.parts.size() &&
                (element.arity == 3 || element.arity == 4),"CIN declared mechanical master is not a source shell");
        native_search::CinAttachmentDeclaration value;
        value.original_nsv_row = original;
        value.ordered_master_rank = rank;
        value.master_source = {native_search::CinMasterSourceKind::DeclaredShellElement,
            element.id,declaration.parts[element.part_index].id};
        // The mechanical patch comes from declaration/IRECT, not the optional
        // coincident INCOQ material/thickness witness in geometry.matches.
        value.topology = element.arity == 3 ? native_search::CinMasterTopology::TriangleRepeatedThird :
                                            native_search::CinMasterTopology::Quad;
        const auto secondary = geometry.secondary_working_nodes[original];
        value.secondary_source_id = source.Id(geometry,secondary);
        Require(value.secondary_source_id == declaration.slave_nodes[original].id,
                "CIN secondary source NID changed");
        value.reference_positions[0] = source.Position(geometry,secondary);
        for (std::size_t slot = 0; slot < 4; ++slot) {
            value.master_source_ids[slot] = source.Id(geometry,master.working_nodes[slot]);
            value.reference_positions[slot+1] = source.Position(geometry,master.working_nodes[slot]);
        }
        out.push_back(value);
    }
    return out;
}
}
