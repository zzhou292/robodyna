#include "MappingFields.h"
#include "MappingArrays.h"
#include "InputChecks.h"
#include "SourceAuthority.h"
#include <set>

namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
} // namespace
Document MappingDocument(const CanonicalSource& source, const std::string& digest,
        const std::array<arrays::Descriptor, 8>& descriptors,const MappingExecution* execution) {
    arrays::CheckHash(digest);
    Document d; d.SetObject();
    String(d, "schema", execution?"robo_dyna.full_shell_source_mapping.v2":MappingSchema);
    if(execution)Child(d,"execution",MappingExecutionDocument(*execution));
    String(d, "mapping_sha256", digest);
    String(d, "scope", "immutable_source_mapping_only_native_meaning_and_runtime_admission_external");
    Child(d, "source_authority", AuthorityDocument(source.data().inputs));
    Document values; values.SetObject();
    const arrays::Limits cap{source.data().limits.file_bytes, UINT32_MAX, 64};
    for (std::size_t i = 0; i < descriptors.size(); ++i)
        Child(values, MappingSpecs()[i].name, arrays::DescriptorDocument(descriptors[i], cap));
    Child(d, "arrays", values);
    return d;
}
std::array<arrays::Descriptor, 8> ParseMappingDocument(const CanonicalSource& source,
        const Value& v, const std::string& expected_digest,MappingExecution* execution) {
    arrays::CheckHash(expected_digest);
    const auto schema = Text(Field(v, "schema"));
    if (schema == "robo_dyna.full_shell_source_mapping.v2") {
        Keys(v, {"schema", "mapping_sha256", "scope", "source_authority", "arrays", "execution"});
        Require(execution, "Mapping-v2 requires execution provenance readback");
        *execution = ParseMappingExecution(v["execution"]);
    } else {
        Keys(v, {"schema", "mapping_sha256", "scope", "source_authority", "arrays"});
        Require(schema == MappingSchema, "Unknown source mapping schema");
        if (execution) *execution = {};
    }
    Require(Text(v["mapping_sha256"]) == expected_digest &&
        Text(v["scope"]) == "immutable_source_mapping_only_native_meaning_and_runtime_admission_external",
        "Mapping schema/digest/scope mismatch");
    CheckAuthority(source.data().inputs, v["source_authority"]);
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
