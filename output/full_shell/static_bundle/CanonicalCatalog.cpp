#include "InputChecks.h"
#include "CanonicalSpecs.h"
#include <algorithm>
#include <charconv>
#include <map>
#include <set>

namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
struct Section { unsigned elform = 0; bool shell = false; };
std::uint64_t Positive(const Value& v) {
    const auto value = UInt(v);
    Require(value, "Zero source declaration ID");
    return value;
}
unsigned SourceElform(const Value& v) {
    const auto text = Text(v);
    unsigned value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    Require(parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size(),
        "Nonintegral source shell formulation field");
    return value; // A literal zero remains zero; defaults are not invented.
}
void Parts(CanonicalData& out, const Value& doc) {
    const auto& materials = Field(doc, "materials");
    const auto& sections = Field(doc, "sections");
    const auto& parts = Field(doc, "parts");
    Require(materials.IsArray() && materials.Size() <= 65536 && sections.IsArray() &&
        sections.Size() <= 65536 && parts.IsArray() && parts.Size() <= 65536,
        "Canonical declaration table exceeds capacity");
    std::set<std::uint64_t> material_ids;
    for (const auto& m : materials.GetArray())
        Require(material_ids.insert(Positive(Field(m, "source_material_id"))).second,
            "Duplicate source material ID");
    std::map<std::uint64_t, Section> section_ids;
    for (const auto& s : sections.GetArray()) {
        const bool shell = Text(Field(s, "keyword")) == "*SECTION_SHELL";
        const Section value{shell ? SourceElform(Field(s, "formulation_field_raw")) : 0, shell};
        Require(section_ids.emplace(Positive(Field(s, "source_section_id")), value).second,
            "Duplicate source section ID");
    }
    std::map<std::uint64_t, PartDeclaration> ordered;
    for (const auto& p : parts.GetArray()) {
        PartDeclaration value;
        value.part = Positive(Field(p, "source_part_id"));
        value.material = Positive(Field(p, "source_material_id"));
        value.section = Positive(Field(p, "source_section_id"));
        const auto section = section_ids.find(value.section);
        Require(material_ids.count(value.material) && section != section_ids.end(),
            "Source part has missing material/section declaration");
        value.source_elform = section->second.elform;
        value.shell_section = section->second.shell;
        const auto& raw = Field(p, "raw_fields");
        Require(raw.IsArray() && raw.Size() == 8 && UInt(raw[0]) == value.part &&
            UInt(raw[1]) == value.section && UInt(raw[2]) == value.material,
            "Canonical part IDs differ from raw source fields");
        Require(ordered.emplace(value.part, value).second, "Duplicate source part ID");
    }
    for (const auto& p : ordered) out.parts.push_back(p.second);
}
} // namespace
void ReadCatalog(CanonicalData& out, const Value& doc) {
    using namespace array_json;
    Require(Text(Field(doc, "schema")) == CanonicalSchema &&
        Field(doc, "canonical_geometry_validated").IsTrue() && Field(doc, "simulation_ready").IsFalse() &&
        Field(doc, "applied_rigid_transform").IsNull() && Text(Field(doc, "output_length_unit")) == "m" &&
        Text(Field(doc, "source_length_unit")) == out.inputs.units.length &&
        Bits(Real(Field(doc, "length_scale"))) == Bits(out.inputs.units.length_to_m),
        "Canonical source schema/units/transform scope mismatch");
    Parts(out, doc);
    out.archive_sha256 = Text(Field(Field(doc, "source_archive"), "sha256"));
    arrays::CheckHash(out.archive_sha256);
    const auto& counts = Field(doc, "counts");
    const auto nodes = UInt(Field(counts, "nodes")), shells = UInt(Field(counts, "shells"));
    const auto solids = UInt(Field(counts, "solids")), beams = UInt(Field(counts, "beams"));
    Require(nodes && nodes <= out.limits.nodes && shells && shells <= out.limits.parents &&
        solids <= out.limits.parents && beams <= out.limits.parents &&
        UInt(Field(counts, "parts")) == out.parts.size(), "Canonical source count mismatch");
    out.canonical_nodes = static_cast<std::size_t>(nodes);
    out.canonical_shells = static_cast<std::size_t>(shells);
    ReadCanonicalArrays(out,Field(doc,"arrays"),nodes,shells,solids,beams);
}
void ReadCanonicalArrays(CanonicalData& out,const Value& table,std::size_t nodes,
    std::size_t shells,std::size_t solids,std::size_t beams) {
    using namespace array_json;
    Keys(table, {"node_ids", "node_positions", "node_codes", "node_blank_masks", "node_source_lines",
        "shells_records", "shells_node_indices", "shells_source_lines", "shells_blank_masks",
        "solids_records", "solids_node_indices", "solids_source_lines", "solids_blank_masks",
        "beams_records", "beams_node_indices", "beams_source_lines", "beams_blank_masks"});
    const arrays::Limits limits{out.limits.file_bytes, std::max(out.limits.nodes, out.limits.parents), 64};
    std::size_t bytes = 0, index = 0;
    std::set<std::string> files;
    for (const auto& spec : CanonicalSpecs()) {
        auto& a = out.arrays[index++];
        a.name = spec.name;
        a.descriptor = arrays::ParseDescriptor(Field(table, spec.name), limits);
        const auto& layout = a.descriptor.layout;
        const auto rows = a.name.rfind("node_", 0) == 0 ? nodes :
            a.name.rfind("shells_", 0) == 0 ? shells : a.name.rfind("solids_", 0) == 0 ? solids : beams;
        Require(layout.scalar == spec.scalar && layout.rows == rows && layout.columns == spec.columns &&
            layout.fields == FieldNames(spec.fields) && files.insert(a.descriptor.file).second,
            "Canonical array layout/field/name association changed");
        AddBytes(bytes, a.descriptor.bytes, out.limits.canonical_array_bytes);
    }
}
} // namespace crash::output::full_shell::source::detail
