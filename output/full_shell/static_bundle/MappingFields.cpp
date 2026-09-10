#include "MappingFields.h"
#include "MappingArrays.h"
#include "InputChecks.h"
#include <set>

namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
Document Authority(const CanonicalSource& source) {
    const auto& in = source.data().inputs;
    Document d; d.SetObject();
    String(d, "canonical_manifest_sha256", in.canonical_manifest.sha256);
    Integer(d, "canonical_manifest_bytes", in.canonical_manifest.bytes);
    String(d, "scope_report_sha256", in.scope_report.sha256);
    Integer(d, "scope_report_bytes", in.scope_report.bytes);
    String(d, "source_member_sha256", in.source_member.sha256);
    Integer(d, "source_member_bytes", in.source_member.bytes);
    String(d, "tire_policy", in.tire_policy);
    String(d, "source_mass_unit", in.units.mass);
    String(d, "source_length_unit", in.units.length);
    String(d, "source_time_unit", in.units.time);
    Number(d, "mass_to_kg", in.units.mass_to_kg);
    Number(d, "length_to_m", in.units.length_to_m);
    Number(d, "time_to_s", in.units.time_to_s);
    return d;
}
void CheckAuthority(const CanonicalSource& source, const Value& v) {
    Keys(v, {"canonical_manifest_sha256", "canonical_manifest_bytes", "scope_report_sha256",
        "scope_report_bytes", "source_member_sha256", "source_member_bytes", "tire_policy",
        "source_mass_unit", "source_length_unit", "source_time_unit", "mass_to_kg", "length_to_m", "time_to_s"});
    const auto& in = source.data().inputs;
    const Units units{Text(v["source_mass_unit"]), Text(v["source_length_unit"]), Text(v["source_time_unit"]),
        Real(v["mass_to_kg"]), Real(v["length_to_m"]), Real(v["time_to_s"])};
    Require(Text(v["canonical_manifest_sha256"]) == in.canonical_manifest.sha256 &&
        UInt(v["canonical_manifest_bytes"]) == in.canonical_manifest.bytes &&
        Text(v["scope_report_sha256"]) == in.scope_report.sha256 && UInt(v["scope_report_bytes"]) == in.scope_report.bytes &&
        Text(v["source_member_sha256"]) == in.source_member.sha256 && UInt(v["source_member_bytes"]) == in.source_member.bytes &&
        Text(v["tire_policy"]) == in.tire_policy && SameUnits(units, in.units),
        "Mapping record does not match caller source authority");
}
} // namespace
Document MappingDocument(const CanonicalSource& source, const std::string& digest,
        const std::array<arrays::Descriptor, 8>& descriptors) {
    arrays::CheckHash(digest);
    Document d; d.SetObject();
    String(d, "schema", MappingSchema);
    String(d, "mapping_sha256", digest);
    String(d, "scope", "immutable_source_mapping_only_native_meaning_and_runtime_admission_external");
    Child(d, "source_authority", Authority(source));
    Document values; values.SetObject();
    const arrays::Limits cap{source.data().limits.file_bytes, UINT32_MAX, 64};
    for (std::size_t i = 0; i < descriptors.size(); ++i)
        Child(values, MappingSpecs()[i].name, arrays::DescriptorDocument(descriptors[i], cap));
    Child(d, "arrays", values);
    return d;
}
std::array<arrays::Descriptor, 8> ParseMappingDocument(const CanonicalSource& source,
        const Value& v, const std::string& expected_digest) {
    arrays::CheckHash(expected_digest);
    Keys(v, {"schema", "mapping_sha256", "scope", "source_authority", "arrays"});
    Require(Text(v["schema"]) == MappingSchema && Text(v["mapping_sha256"]) == expected_digest &&
        Text(v["scope"]) == "immutable_source_mapping_only_native_meaning_and_runtime_admission_external",
        "Mapping schema/digest/scope mismatch");
    CheckAuthority(source, v["source_authority"]);
    const auto& input = v["arrays"];
    Keys(input, {"node_source_ids", "node_canonical_indices", "parent_source_ids", "parent_reference",
        "parent_points", "parent_nodes", "triangles", "triangle_parents"});
    std::array<arrays::Descriptor, 8> result;
    const auto& s = source.data();
    const arrays::Limits cap{s.limits.file_bytes, UINT32_MAX, 64};
    std::set<std::string> names;
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = arrays::ParseDescriptor(input[MappingSpecs()[i].name], cap);
        const auto wanted = MappingLayout(i, s.retained_nodes, s.retained_shells, 2 * s.retained_q4 + s.retained_t3);
        const auto& actual = result[i].layout;
        Require(actual.scalar == wanted.scalar && actual.rows == wanted.rows && actual.columns == wanted.columns &&
            actual.fields == wanted.fields && names.insert(result[i].file).second,
            "Mapping array source count/layout/path mismatch");
    }
    return result;
}
} // namespace crash::output::full_shell::source::detail
