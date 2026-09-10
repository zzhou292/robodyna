#include "PreparedSourceMapping.h"
#include "MappingDraft.h"

namespace crash::output::full_shell::source {
struct PreparedSourceMapping::Data {
    explicit Data(const CanonicalSource& value) : source(value) {}
    CanonicalSource source;
    std::array<NamedArray, 8> arrays;
    std::vector<ParentPoints> points;
    std::string digest;
    std::size_t bytes = 0;
};
PreparedSourceMapping PreparedSourceMapping::Prepare(const CanonicalSource& source, MappingInput input) {
    auto draft = detail::BuildMapping(source.data(), input);
    auto data = std::make_shared<Data>(source);
    const arrays::Limits limits{source.data().limits.file_bytes, UINT32_MAX, 64};
    data->arrays = detail::EncodeMapping(draft, limits);
    data->points = std::move(draft.points);
    data->digest = MappingDigest(data->arrays, limits);
    for (const auto& a : data->arrays) data->bytes += a.descriptor.bytes;
    return PreparedSourceMapping(std::move(data));
}
const CanonicalSource& PreparedSourceMapping::source() const noexcept { return data_->source; }
const std::array<NamedArray, 8>& PreparedSourceMapping::arrays() const noexcept { return data_->arrays; }
const std::string& PreparedSourceMapping::digest() const noexcept { return data_->digest; }
const std::vector<ParentPoints>& PreparedSourceMapping::parents() const noexcept { return data_->points; }
std::size_t PreparedSourceMapping::nodes() const noexcept { return data_->source.data().retained_nodes; }
std::size_t PreparedSourceMapping::triangles() const noexcept { return data_->arrays[detail::Triangles].descriptor.layout.rows; }
std::size_t PreparedSourceMapping::payload_bytes() const noexcept { return data_->bytes; }
Context PreparedSourceMapping::MakeFrameContext(Identity id, double dt, RecordLimits limits) const {
    const auto& report = source().data().inputs.scope_report;
    Require((!id.source_inventory_bytes || id.source_inventory_bytes == report.bytes) &&
        (id.source_inventory_sha256.empty() || id.source_inventory_sha256 == report.sha256) &&
        (id.source_mapping_sha256.empty() || id.source_mapping_sha256 == digest()),
        "Caller frame identity conflicts with prepared source mapping");
    id.source_inventory_bytes = report.bytes;
    id.source_inventory_sha256 = report.sha256;
    id.source_mapping_sha256 = digest();
    return Context::Create(std::move(id), nodes(), parents().data(), parents().size(), dt, limits);
}
} // namespace crash::output::full_shell::source
