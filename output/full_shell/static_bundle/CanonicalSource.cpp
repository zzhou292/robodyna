#include "InputChecks.h"
#include "CanonicalSpecs.h"

namespace crash::output::full_shell::source {
namespace {
std::shared_ptr<const CanonicalData> ReadSource(const SourceInputs& in, SourceLimits limits,
        const std::string* member_bytes) {
    std::size_t budget = detail::SourceReadBudget(in, limits);
    if (member_bytes) {
        Require(member_bytes->size() == in.source_member.bytes &&
            Sha256(*member_bytes) == in.source_member.sha256, "Source member bytes differ from authority");
    }
    auto data = std::make_shared<CanonicalData>();
    data->inputs = in;
    data->limits = limits;
    data->canonical_bytes = detail::ReadFile(in.canonical_root, in.canonical_manifest, limits.file_bytes);
    data->scope_bytes = detail::ReadFile(in.scope_root, in.scope_report, limits.file_bytes);
    auto canonical = array_json::Parse(data->canonical_bytes, limits.file_bytes);
    const bool declared = array_json::Text(detail::Field(canonical,"schema"))==DeclaredCanonicalSchema;
    if(declared)detail::ReadDeclaredCatalog(*data,canonical);
    else detail::ReadCatalog(*data, canonical);
    auto scope = array_json::Parse(data->scope_bytes, limits.file_bytes);
    if(declared)detail::ReadDeclaredScope(*data,scope);
    else detail::ReadScope(*data, scope, canonical);
    for (const auto& a : data->arrays) detail::AddBytes(budget, 4 * a.descriptor.bytes, limits.host_bytes);
    // Source membership is authenticated as complete original bytes; no keyword
    // interpretation occurs here. The later copy layer rechecks it before copy.
    if (!member_bytes) detail::ReadFile(in.member_root, in.source_member, limits.source_member_bytes);
    const arrays::Limits array_limits{limits.file_bytes, limits.nodes > limits.parents ? limits.nodes : limits.parents, 64};
    for (auto& a : data->arrays) a.bytes = arrays::ReadBytes(in.canonical_root, a.descriptor, array_limits);
    detail::CheckCanonicalGeometry(*data);
    if(declared)detail::CheckDeclaredAnnotations(*data);
    return data;
}
} // namespace
CanonicalSource CanonicalSource::Read(const SourceInputs& in, SourceLimits limits) {
    return CanonicalSource(ReadSource(in, limits, nullptr));
}
CanonicalSource CanonicalSource::ReadWithMemberBytes(const SourceInputs& in,
        const std::string& bytes, SourceLimits limits) {
    return CanonicalSource(ReadSource(in, limits, &bytes));
}
} // namespace crash::output::full_shell::source
