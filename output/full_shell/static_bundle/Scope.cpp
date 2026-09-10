#include "InputChecks.h"
#include <algorithm>

namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
std::vector<std::uint64_t> PartIds(const Value& v) {
    Require(v.IsArray() && v.Size() <= 65536, "Scope part list exceeds capacity");
    std::vector<std::uint64_t> ids;
    ids.reserve(v.Size());
    for (const auto& x : v.GetArray()) {
        const auto id = UInt(x);
        Require(id && (ids.empty() || id > ids.back()), "Scope part IDs are not unique and ordered");
        ids.push_back(id);
    }
    return ids;
}
bool SameArray(const arrays::Descriptor& a, const arrays::Descriptor& b) {
    return a.file == b.file && a.bytes == b.bytes && a.sha256 == b.sha256 &&
        a.layout.scalar == b.layout.scalar && a.layout.rows == b.layout.rows &&
        a.layout.columns == b.layout.columns && a.layout.fields == b.layout.fields;
}
std::size_t Count(const Value& v, const char* name, std::size_t cap) {
    const auto n = UInt(Field(v, name));
    Require(n <= cap, "Source coverage count exceeds capacity");
    return static_cast<std::size_t>(n);
}
} // namespace
void ReadScope(CanonicalData& d, const Value& doc, const Value& canonical) {
    Require(Text(Field(doc, "schema")) == ScopeSchema && Field(doc, "simulation_ready").IsFalse() &&
        Field(doc, "geometry_modified").IsFalse() && Field(doc, "source_mass_equivalence_qualified").IsFalse() &&
        Text(Field(doc, "tire_policy")) == d.inputs.tire_policy,
        "Source scope schema/policy/admission mismatch");
    const auto& u = Field(doc, "units");
    Keys(u, {"mass", "length", "time", "mass_to_kg", "length_to_m", "time_to_s"});
    const Units units{Text(u["mass"]), Text(u["length"]), Text(u["time"]),
        Real(u["mass_to_kg"]), Real(u["length_to_m"]), Real(u["time_to_s"])};
    CheckUnits(units);
    Require(SameUnits(units, d.inputs.units), "Source units differ from caller authority");
    const auto& source = Field(doc, "source");
    Require(Text(Field(source, "archive_sha256")) == d.archive_sha256 &&
        Text(Field(source, "member_sha256")) == d.inputs.source_member.sha256 &&
        UInt(Field(source, "member_bytes")) == d.inputs.source_member.bytes,
        "Original source member/archive identity mismatch");
    const auto& unit_authority = Field(source, "unit_authority");
    Require(Sha256(Text(Field(unit_authority, "raw_text"))) == Text(Field(unit_authority, "sha256")),
        "Source unit authority text hash mismatch");
    const auto member = Text(Field(source, "archive_member"));
    arrays::CheckRelativeName(member);
    const auto name = std::filesystem::path(member).filename().string();
    Require(Text(Field(Field(Field(canonical, "source_files"), name.c_str()), "sha256")) ==
        d.inputs.source_member.sha256, "Canonical source member identity mismatch");
    const auto& evidence = Field(doc, "canonical");
    Require(Text(Field(evidence, "manifest_sha256")) == d.inputs.canonical_manifest.sha256 &&
        Field(evidence, "source_records_independently_verified").IsTrue(),
        "Canonical manifest differs from authenticated source scope");
    const auto& arrays_value = Field(evidence, "arrays");
    Require(arrays_value.IsObject() && arrays_value.MemberCount() == CanonicalArrayCount,
        "Scope canonical array inventory differs");
    const arrays::Limits limits{d.limits.file_bytes, std::max(d.limits.nodes, d.limits.parents), 64};
    for (const auto& a : d.arrays)
        Require(SameArray(a.descriptor, arrays::ParseDescriptor(Field(arrays_value, a.name.c_str()), limits)),
            "Scope canonical array identity differs");
    d.selected_parts = PartIds(Field(doc, "selected_shell_part_ids"));
    d.excluded_parts = PartIds(Field(doc, "excluded_tire_shell_part_ids"));
    Require(!d.selected_parts.empty() &&
        (d.inputs.tire_policy == "retain_all" ? d.excluded_parts.empty() : !d.excluded_parts.empty()),
        "Source tire policy does not match explicit exclusions");
    for (auto pid : d.selected_parts) {
        Require(!std::binary_search(d.excluded_parts.begin(), d.excluded_parts.end(), pid) &&
            FindPart(d, pid).shell_section, "Selected/excluded part scope overlaps or is not a shell section");
    }
    for (auto pid : d.excluded_parts) Require(FindPart(d, pid).shell_section, "Excluded part is not a source shell section");
    CheckScopeParts(d, Field(Field(doc, "declarations"), "parts"));
    const auto& coverage = Field(doc, "coverage");
    const auto& retained = Field(coverage, "retained");
    const auto& excluded = Field(coverage, "excluded");
    d.retained_nodes = Count(retained, "nodes", d.limits.nodes);
    d.retained_shells = Count(retained, "shells", d.limits.parents);
    d.retained_q4 = Count(retained, "q4", d.retained_shells);
    d.retained_t3 = Count(retained, "t3", d.retained_shells);
    d.excluded_shells = Count(excluded, "shells", d.canonical_shells);
    d.excluded_q4 = Count(excluded, "q4", d.excluded_shells);
    d.excluded_t3 = Count(excluded, "t3", d.excluded_shells);
    Require(d.retained_nodes && d.retained_shells && d.retained_shells <= d.canonical_shells &&
        d.excluded_shells == d.canonical_shells - d.retained_shells &&
        d.retained_q4 == d.retained_shells - d.retained_t3 &&
        d.excluded_q4 == d.excluded_shells - d.excluded_t3 &&
        Count(retained, "parts", 65536) == d.selected_parts.size() &&
        Count(excluded, "parts", 65536) == d.excluded_parts.size(), "Source coverage totals differ");
    d.retained_shell_ids_sha256 = Text(Field(coverage, "retained_shell_ids_sha256"));
    d.excluded_shell_ids_sha256 = Text(Field(coverage, "excluded_shell_ids_sha256"));
    d.selected_node_ids_sha256 = Text(Field(coverage, "selected_node_ids_sha256"));
    for (const auto* hash : {&d.retained_shell_ids_sha256, &d.excluded_shell_ids_sha256, &d.selected_node_ids_sha256})
        arrays::CheckHash(*hash);
}
} // namespace crash::output::full_shell::source::detail
