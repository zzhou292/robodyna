#pragma once
#include "SourceBundle.h"
#include "InputChecks.h"

namespace crash::output::full_shell::source {
struct PreparedSourceBundle::Data {
    explicit Data(const PreparedSourceMapping& value) : mapping(value) {}
    PreparedSourceMapping mapping;
    BundleDescription description;
    MappingRecordPlan mapping_plan;
    RecordFile descriptor;
    std::string member_bytes, metadata_bytes;
    Plan archive_plan;
};
namespace detail {
void CheckBundleStem(const std::string&);
bool SameFile(const RecordFile&, const RecordFile&) noexcept;
RecordFile FindBundleFile(const BundleDescription&, const std::string&);
std::vector<SourceChunk> DescribeChunks(const std::string& stem, const std::string& bytes,
    std::size_t file_cap);
void CheckChunks(const std::vector<SourceChunk>&, std::size_t member_bytes, std::size_t file_cap);
Document BundleDocument(const SourceInputs&, const BundleDescription&);
BundleDescription ParseBundleDocument(const Value&, const SourceInputs&, const std::string& digest,
    const RecordFile& descriptor, SourceLimits);
std::string EncodeBundleMetadata(const SourceInputs&, BundleDescription&);
std::vector<RecordFile> Inventory(const PreparedSourceMapping&, const MappingRecordPlan&,
    const BundleDescription&);
void CheckInventory(const std::vector<RecordFile>&, const std::vector<RecordFile>&);
} // namespace detail
} // namespace crash::output::full_shell::source
