#include "InputChecks.h"
#include "CanonicalSpecs.h"

namespace crash::output::full_shell::source {
CanonicalSource CanonicalSource::Read(const SourceInputs& in, SourceLimits limits) {
    Require(limits.file_bytes && limits.file_bytes <= kArtifactFileCap &&
        limits.source_member_bytes && limits.source_member_bytes <= 64 * 1024 * 1024 &&
        limits.canonical_array_bytes && limits.canonical_array_bytes <= 64 * 1024 * 1024 &&
        limits.host_bytes && limits.host_bytes <= 512 * 1024 * 1024 &&
        limits.nodes && limits.nodes <= 1048576 && limits.parents && limits.parents <= 1048576,
        "Invalid static source resource limits");
    CheckUnits(in.units);
    Require(in.tire_policy == "retain_all" || in.tire_policy == "omit_original_tire_shells",
        "Undeclared source tire policy");
    Require(in.canonical_manifest.bytes && in.canonical_manifest.bytes <= limits.file_bytes &&
        in.scope_report.bytes && in.scope_report.bytes <= limits.file_bytes &&
        in.source_member.bytes && in.source_member.bytes <= limits.source_member_bytes,
        "Source input byte declarations exceed capacity");
    std::size_t budget = 0;
    detail::AddBytes(budget, 4 * in.canonical_manifest.bytes, limits.host_bytes);
    detail::AddBytes(budget, 4 * in.scope_report.bytes, limits.host_bytes);
    detail::AddBytes(budget, 2 * in.source_member.bytes, limits.host_bytes);
    auto data = std::make_shared<CanonicalData>();
    data->inputs = in;
    data->limits = limits;
    data->canonical_bytes = detail::ReadFile(in.canonical_root, in.canonical_manifest, limits.file_bytes);
    data->scope_bytes = detail::ReadFile(in.scope_root, in.scope_report, limits.file_bytes);
    auto canonical = array_json::Parse(data->canonical_bytes, limits.file_bytes);
    detail::ReadCatalog(*data, canonical);
    auto scope = array_json::Parse(data->scope_bytes, limits.file_bytes);
    detail::ReadScope(*data, scope, canonical);
    for (const auto& a : data->arrays) detail::AddBytes(budget, 4 * a.descriptor.bytes, limits.host_bytes);
    // Source membership is authenticated as complete original bytes; no keyword
    // interpretation occurs here. The later copy layer rechecks it before copy.
    detail::ReadFile(in.member_root, in.source_member, limits.source_member_bytes);
    const arrays::Limits array_limits{limits.file_bytes, limits.nodes > limits.parents ? limits.nodes : limits.parents, 64};
    for (auto& a : data->arrays) a.bytes = arrays::ReadBytes(in.canonical_root, a.descriptor, array_limits);
    detail::CheckCanonicalGeometry(*data);
    return CanonicalSource(std::move(data));
}
} // namespace crash::output::full_shell::source
