#include "SourceBundleInternal.h"

namespace crash::output::full_shell::source {
PreparedSourceBundle PreparedSourceBundle::Prepare(const PreparedSourceMapping& mapping, const BundleRequest& request) {
    detail::CheckBundleStem(request.stem);
    const auto& source = mapping.source().data();
    const auto& archive = request.archive;
    std::size_t points = 0;
    for (const auto& p : mapping.parents())
        if (p.plastic == PlasticField::NativeEquivalentPlasticStrain)
            detail::AddBytes(points, p.native_points, 4194304);
    Require(archive.nodes == mapping.nodes() && archive.parents == mapping.parents().size() &&
        archive.plastic_points == points &&
        archive.static_byte_reserve <= StaticReserveBytes && archive.file_byte_cap &&
        archive.file_byte_cap <= source.limits.file_bytes,
        "Static bundle does not match planned frame counts or capacities");
    auto data = std::make_shared<Data>(mapping);
    auto& d = data->description;
    d.stem = request.stem;
    d.mapping_sha256 = mapping.digest();
    d.file_byte_cap = archive.file_byte_cap;
    d.canonical_manifest = {d.stem + "-canonical.json", source.inputs.canonical_manifest.sha256,
        source.canonical_bytes.size()};
    d.scope_report = {d.stem + "-scope.json", source.inputs.scope_report.sha256, source.scope_bytes.size()};
    data->mapping_plan = PlanMappingRecord(mapping, d.stem + "-mapping");
    d.mapping = data->mapping_plan.description;
    // Preflight the complete known payload and each original array before reading
    // the bounded key again. Exact descriptor bytes are added below, before I/O.
    std::size_t known = source.inputs.source_member.bytes;
    detail::AddBytes(known, source.canonical_bytes.size(), archive.static_byte_reserve);
    detail::AddBytes(known, source.scope_bytes.size(), archive.static_byte_reserve);
    detail::AddBytes(known, data->mapping_plan.bytes, archive.static_byte_reserve);
    Require(source.canonical_bytes.size() <= d.file_byte_cap && source.scope_bytes.size() <= d.file_byte_cap,
        "Original metadata does not fit source file cap");
    for (const auto& a : source.arrays) {
        Require(a.descriptor.bytes <= d.file_byte_cap, "Original canonical array exceeds source file cap");
        detail::AddBytes(known, a.descriptor.bytes, archive.static_byte_reserve);
    }
    for (const auto& f : data->mapping_plan.files)
        Require(f.bytes <= d.file_byte_cap, "Original mapping array exceeds source file cap");
    data->member_bytes = detail::ReadFile(source.inputs.member_root, source.inputs.source_member,
        source.limits.source_member_bytes);
    d.chunks = detail::DescribeChunks(d.stem, data->member_bytes, d.file_byte_cap);
    d.files = detail::Inventory(mapping, data->mapping_plan, d);
    data->metadata_bytes = detail::EncodeBundleMetadata(source.inputs, d);
    data->descriptor = {d.stem + ".bundle.json", Sha256(data->metadata_bytes), data->metadata_bytes.size()};
    Require(data->descriptor.bytes <= d.file_byte_cap, "Static bundle descriptor exceeds file cap");
    PlanRequest merged = archive;
    for (const auto& f : d.files) merged.static_files.push_back({f.file, f.bytes});
    merged.static_files.push_back({data->descriptor.file, data->descriptor.bytes});
    data->archive_plan = PlanArchive(merged);
    return PreparedSourceBundle(std::move(data));
}
const PreparedSourceMapping& PreparedSourceBundle::mapping() const noexcept { return data_->mapping; }
const BundleDescription& PreparedSourceBundle::description() const noexcept { return data_->description; }
const RecordFile& PreparedSourceBundle::descriptor() const noexcept { return data_->descriptor; }
const Plan& PreparedSourceBundle::archive_plan() const noexcept { return data_->archive_plan; }
std::vector<FileReservation> PreparedSourceBundle::reservations() const {
    std::vector<FileReservation> files;
    files.reserve(data_->description.files.size() + 1);
    for (const auto& f : data_->description.files) files.push_back({f.file, f.bytes});
    files.push_back({data_->descriptor.file, data_->descriptor.bytes});
    return files;
}
} // namespace crash::output::full_shell::source
