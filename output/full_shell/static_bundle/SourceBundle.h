#pragma once
#include "MappingRecords.h"

namespace crash::output::full_shell::source {
inline constexpr const char* BundleSchema = "robo_dyna.full_shell_static_source_bundle.v1";
inline constexpr std::size_t BundleMetadataByteCap = 32 * 1024;
struct SourceChunk { std::size_t offset = 0; RecordFile record; };
struct BundleRequest {
    std::string stem = "source";
    // Explicit later wall/configuration/index/run-manifest obligations belong
    // here. Prepare appends all static source files, never infers completeness.
    PlanRequest archive;
};
struct BundleDescription {
    std::string stem, mapping_sha256;
    RecordFile canonical_manifest, scope_report, mapping;
    std::vector<SourceChunk> chunks;
    std::vector<RecordFile> files; // Exact ordered inventory, excludes descriptor.
    std::size_t file_byte_cap = kArtifactFileCap, static_bytes = 0;
};
class PreparedSourceBundle {
  public:
    static PreparedSourceBundle Prepare(const PreparedSourceMapping&, const BundleRequest&);
    PreparedSourceBundle(const PreparedSourceBundle&) noexcept = default;
    PreparedSourceBundle(PreparedSourceBundle&& other) noexcept : data_(other.data_) {}
    PreparedSourceBundle& operator=(const PreparedSourceBundle&) = delete;
    PreparedSourceBundle& operator=(PreparedSourceBundle&&) = delete;
    const PreparedSourceMapping& mapping() const noexcept;
    const BundleDescription& description() const noexcept;
    const RecordFile& descriptor() const noexcept;
    const Plan& archive_plan() const noexcept;
    std::vector<FileReservation> reservations() const;
  private:
    struct Data;
    explicit PreparedSourceBundle(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
    friend RecordFile WriteSourceBundle(const std::filesystem::path&, const PreparedSourceBundle&);
};
// Source arrays keep their canonical paths. The caller creates the real root
// and arrays/ directory, and serializes all writes. No directory, run completion
// receipt, live owner, clock, or mechanics state is constructed by this layer.
RecordFile WriteSourceBundle(const std::filesystem::path&, const PreparedSourceBundle&);
// The exact descriptor identity plus original source/units/selection and mapping
// identity come from caller authority, normally an authenticated outer manifest.
// Staging returns a complete immutable mapping only after all files/semantics
// pass. This is a static bundle, not a restart or simulation validation result.
PreparedSourceMapping ReadSourceBundle(const std::filesystem::path&, const RecordFile& descriptor,
    const SourceInputs& expected_source, const std::string& expected_mapping_sha256, SourceLimits = {});
} // namespace crash::output::full_shell::source
