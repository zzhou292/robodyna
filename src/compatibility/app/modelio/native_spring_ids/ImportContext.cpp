#include "Internal.h"
#include <algorithm>
namespace crash::modelio::native_spring_ids {
namespace detail {
void CheckLimits(Limits limits) {
    const Limits hard;
    const std::size_t values[]{limits.host_bytes, limits.member_bytes, limits.metadata_bytes,
        limits.members, limits.blocks, limits.rows, limits.resolve_bytes};
    const std::size_t maxima[]{hard.host_bytes, hard.member_bytes, hard.metadata_bytes,
        hard.members, hard.blocks, hard.rows, hard.resolve_bytes};
    for (unsigned i = 0; i < std::size(values); ++i)
        if (!values[i] || values[i] > maxima[i]) Reject(Readiness::ResourceLimit, "Invalid native SPRING identity limits");
}
}
namespace {
void Add(std::size_t& bytes, std::size_t count, std::size_t width = 1) {
    if (!width || count > (SIZE_MAX-bytes)/width) detail::Reject(Readiness::ResourceLimit, "SPRING identity forecast overflow");
    bytes += count*width;
}
}
struct ImportContext::Storage {
    explicit Storage(const source::CanonicalSource& value) : canonical(value) {}
    source::CanonicalSource canonical;
    ContextData data;
};
Forecast ImportContext::Preflight(const source::CanonicalSource& source, const ImportMembers& input, Limits limits) {
    detail::CheckLimits(limits);
    if (input.profile != Profile::DirectKeywordR14FreshRadiossPoSortById)
        detail::Reject(Readiness::UnsupportedProfile, "Unresolved direct-key reader/backend/selection profile");
    if (input.members.empty() || input.members.size() > limits.members)
        detail::Reject(Readiness::ResourceLimit, "Import member count exceeds cap");
    const auto& canonical = source.data();
    Forecast f;
    f.retained_source = canonical.limits.host_bytes;
    std::size_t member_bytes = 0;
    for (const auto& member : input.members) {
        if (member.filename.size() > 128 || member.bytes.size() > limits.member_bytes-member_bytes)
            detail::Reject(Readiness::ResourceLimit, "Import member bytes exceed cap");
        member_bytes += member.bytes.size();
    }
    // Borrowed members plus one simultaneous hash/block extraction copy.
    Add(f.member_reservation, 2, member_bytes);
    Add(f.decode_scratch, 8, canonical.canonical_bytes.capacity()+1);
    for (const auto* name : {"beams_records", "beams_source_lines"})
        Add(f.decode_scratch, 2, source::FindArray(canonical, name).descriptor.bytes);
    Add(f.decode_scratch, limits.blocks, 512);
    f.result_reservation = sizeof(Storage);
    Add(f.result_reservation, 2*limits.rows, sizeof(SourceRow)+128);
    Add(f.result_reservation, 16, limits.metadata_bytes);
    Add(f.result_reservation, limits.members, sizeof(output::full_shell::RecordFile)+256);
    f.total_bytes = f.retained_source;
    Add(f.total_bytes, f.member_reservation); Add(f.total_bytes, f.decode_scratch); Add(f.total_bytes, f.result_reservation);
    if (f.total_bytes > limits.host_bytes) detail::Reject(Readiness::ResourceLimit, "Complete import-context reservation exceeds cap");
    return f;
}
ImportContext ImportContext::Prepare(const source::CanonicalSource& canonical, const ImportMembers& input, Limits limits) {
    auto storage = std::make_shared<Storage>(canonical);
    try {
        const auto forecast = Preflight(canonical, input, limits);
        storage->data = detail::BuildContext(canonical.data(), input, limits);
        storage->data.forecast = forecast;
    } catch (const detail::Failure& error) {
        storage->data = {}; storage->data.diagnostic = error.diagnostic;
    } catch (const std::exception& error) {
        storage->data = {}; storage->data.diagnostic = {Readiness::InvalidSource, std::string(error.what()).substr(0,1024), {}, 0, 0};
    }
    return ImportContext(std::move(storage));
}
const source::CanonicalSource& ImportContext::canonical() const noexcept { return storage_->canonical; }
const ContextData& ImportContext::data() const noexcept { return storage_->data; }
} // namespace crash::modelio::native_spring_ids
