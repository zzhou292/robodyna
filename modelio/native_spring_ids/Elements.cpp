#include "Internal.h"
#include "modelio/physical_scope/Internal.h"
#include "modelio/type45/Internal.h"
#include <algorithm>
#include <climits>
#include <map>
#include <set>
namespace crash::modelio::native_spring_ids::detail {
namespace {
void Id(std::uint64_t value, const char* name, const Location& location) {
    if (!value || value > INT_MAX) Reject(Readiness::InvalidSource, name, location.file, location.line, value);
}
template<class T> std::vector<T> Decode(const source::CanonicalData& canonical, const char* name) {
    const auto& value = source::FindArray(canonical, name);
    return output::arrays::Decode<T>(value.descriptor, value.bytes,
        {canonical.limits.file_bytes, std::max(canonical.limits.nodes, canonical.limits.parents), 64});
}
bool Contains(const std::string& text, const char* value) { return text.find(value) != std::string::npos; }
bool SpringBeam(const std::string& material, unsigned elform) {
    const bool null = Contains(material, "*MAT_NULL") || Contains(material, "*MAT_009");
    const bool spot = Contains(material, "*MAT_SPOTWELD") || Contains(material, "*MAT_100");
    const bool muscle = Contains(material, "*MAT_MUSCLE") || Contains(material, "*MAT_156");
    if (null && elform == 6) return true;
    if (null && elform != 0 && elform != 1 && elform != 2) return false; // Native TRUSS arm.
    if (spot || muscle) return true;
    return elform == 6; // Remaining native switch:3 isTRUSS; otherwiseBEAM.
}
}
void PopulateElements(const source::CanonicalData& canonical, const output::Document& document,
        ContextData& data, Limits limits) {
    std::map<std::uint64_t, std::string> materials;
    for (const auto& row : reader::Array(document, "materials", 2048, 1).GetArray()) {
        const auto id = reader::Unsigned(row, "source_material_id");
        if (!materials.emplace(id, reader::Text(row, "keyword")).second)
            Reject(Readiness::InvalidSource, "Duplicate canonical material identity", {}, 0, id);
    }
    std::string main_file;
    for (const auto& member : data.members) if (member.sha256 == canonical.inputs.source_member.sha256) {
        if (!main_file.empty()) Reject(Readiness::InvalidSource, "Ambiguous canonical main member identity");
        main_file = member.file;
    }
    if (main_file.empty()) Reject(Readiness::MissingSource, "Canonical main member is absent from import closure");
    const auto& files = reader::Member(document, "source_files");
    std::size_t declared_beams = 0;
    for (const auto& file : files.GetObject()) {
        const std::string name(file.name.GetString(), file.name.GetStringLength());
        for (const auto& block : reader::Array(file.value, "blocks", limits.blocks, 1).GetArray()) {
            if (reader::Text(block, "keyword") != "*ELEMENT_BEAM") continue;
            if (name != main_file) Reject(Readiness::UnsupportedSource, "Noncanonical member has unclassified beam precursors", name, reader::Unsigned(block, "first_line"));
            const auto count = reader::Unsigned(block, "data_records");
            if (count > limits.rows-declared_beams) Reject(Readiness::ResourceLimit, "Original beam census exceeds cap");
            declared_beams += count;
        }
    }
    const auto beams = Decode<std::uint64_t>(canonical, "beams_records");
    const auto lines = Decode<std::uint32_t>(canonical, "beams_source_lines");
    if (beams.size() != 10*lines.size() || lines.size() > limits.rows || lines.size() != declared_beams)
        Reject(Readiness::ResourceLimit, "Complete original beam extent exceeds ID context");
    std::set<std::uint64_t> ids;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const auto* beam = beams.data()+10*i;
        const Location location{main_file, lines[i]}; Id(beam[0], "Invalid original beam EID", location);
        if (!ids.insert(beam[0]).second) Reject(Readiness::InvalidSource, "Duplicate original beam EID", main_file, lines[i], beam[0]);
        const auto& part = source::FindPart(canonical, beam[1]);
        const auto material = materials.find(part.material);
        if (material == materials.end()) Reject(Readiness::MissingSource, "Beam material identity is unresolved", main_file, lines[i], beam[0]);
        if (part.shell_section) Reject(Readiness::InvalidSource, "Beam is associated with a shell section", main_file, lines[i], beam[0]);
        const auto& keyword = material->second;
        // The converter uses substring tests, but source admission is narrower:
        // only these audited literal aliases enter that branch sequence. No
        // suffixed/custom material is silently admitted by a prefix match.
        const bool known = keyword == "*MAT_SPOTWELD" || keyword == "*MAT_100" ||
            keyword == "*MAT_NULL" || keyword == "*MAT_009" || keyword == "*MAT_MUSCLE" ||
            keyword == "*MAT_156" || keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" ||
            keyword == "*MAT_024" || keyword == "*MAT_ELASTIC" || keyword == "*MAT_001" ||
            keyword == "*MAT_RIGID" || keyword == "*MAT_020";
        if (!known) Reject(Readiness::UnsupportedSource, "Unaudited beam material dispatch", main_file, lines[i], beam[0]);
        if (!SpringBeam(keyword, part.source_elform)) { ++data.non_spring_beams; continue; }
        if (!(Contains(keyword, "*MAT_SPOTWELD") || Contains(keyword, "*MAT_100")) || part.source_elform != 9)
            Reject(Readiness::UnsupportedSource, "Native beam SPRING is outside the existing TYPE13 source policy", main_file, lines[i], beam[0]);
        SourceRow row; row.kind = SourceKind::Type13; row.original_id = beam[0];
        row.endpoints = {beam[2], beam[3]}; row.canonical_index = i; row.location = location;
        Id(row.endpoints[0], "Invalid TYPE13 source endpoint", location);
        Id(row.endpoints[1], "Invalid TYPE13 source endpoint", location);
        data.precursors.push_back(std::move(row));
    }
    for (const auto& evidence : data.evidence) {
        if (evidence.block.keyword != "*ELEMENT_DISCRETE") continue;
        for (const auto& card : evidence.cards) {
            if (assembly::reader::auxiliary::Trim(card.second).empty()) continue;
            if (data.precursors.size() >= limits.rows) Reject(Readiness::ResourceLimit, "SPRING precursor count exceeds cap");
            SourceRow row; row.kind = SourceKind::DiscreteNamespaceOnly;
            row.original_id = assembly::reader::auxiliary::Id(card.second, 0, 8);
            const auto pid = assembly::reader::auxiliary::Id(card.second, 8, 8);
            row.endpoints = {assembly::reader::auxiliary::Id(card.second, 16, 8), assembly::reader::auxiliary::Id(card.second, 24, 8)};
            row.location = {evidence.block.filename, card.first};
            Id(row.original_id, "Invalid discrete precursor EID", row.location); Id(pid, "Invalid discrete precursor PID", row.location);
            Id(row.endpoints[0], "Invalid discrete precursor endpoint", row.location);
            Id(row.endpoints[1], "Invalid discrete precursor endpoint", row.location);
            // The remaining source fields are retained in authenticated raw
            // evidence. They do not alter explicit EID or node1/node2 identity.
            data.precursors.push_back(std::move(row));
        }
    }
    ids.clear();
    for (const auto& row : data.precursors)
        if (!ids.insert(row.original_id).second)
            Reject(Readiness::InvalidSource, "Duplicate explicit ID in the native SPRING namespace", row.location.file, row.location.line, row.original_id);
}
void PopulateConnections(ContextData& data, Limits limits) {
    const auto welds = physical_scope::detail::ReadSpotwelds(data.evidence, {});
    const auto joints = type45::detail::Read(data.evidence, {});
    if (welds.size() > limits.rows || joints.size() > limits.rows ||
        data.precursors.size() > limits.rows-welds.size() || data.precursors.size()+welds.size() > limits.rows-joints.size())
        Reject(Readiness::ResourceLimit, "Complete SPRING namespace exceeds capacity");
    for (const auto& source : welds) {
        const auto& evidence = data.evidence[source.source_index];
        if (!source.default_only) Reject(Readiness::UnsupportedSource, "Additional weld parameter groups are unqualified", evidence.block.filename, evidence.cards[source.first_card].first, source.id);
        SourceRow row; row.kind = SourceKind::DefaultSpotweld; row.original_id = source.id;
        row.endpoints = source.nodes; row.location = {evidence.block.filename, evidence.cards[source.first_card].first};
        Id(row.original_id, "Invalid source WID", row.location);
        for (const auto endpoint : row.endpoints) Id(endpoint, "Invalid weld endpoint", row.location);
        data.welds.push_back(std::move(row));
    }
    for (const auto& source : joints) {
        SourceRow row; row.kind = SourceKind::RegularJoint; row.original_id = source.source_id;
        row.endpoints = {source.nodes[0].source_id, source.nodes[1].source_id};
        row.location = {data.evidence[source.source_index].block.filename, source.header_line};
        Id(row.original_id, "Invalid regular joint ID", row.location);
        for (const auto endpoint : row.endpoints) Id(endpoint, "Invalid joint endpoint", row.location);
        data.joints.push_back(std::move(row));
    }
    // The known PO SortById selection is global across regular joint kinds.
    const auto order = [](const auto& a, const auto& b) { return a.original_id < b.original_id; };
    std::sort(data.welds.begin(), data.welds.end(), order);
    std::sort(data.joints.begin(), data.joints.end(), order);
}
} // namespace crash::modelio::native_spring_ids::detail
