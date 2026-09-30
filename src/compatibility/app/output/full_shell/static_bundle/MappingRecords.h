#pragma once
#include "PreparedSourceMapping.h"
#include "output/full_shell/FullShellVisualizationPlan.h"

namespace crash::output::full_shell::source {
inline constexpr std::size_t MappingMetadataByteCap = 16 * 1024;
struct MappingRecordPlan {
    RecordFile description;
    std::array<arrays::Descriptor, 8> arrays;
    std::vector<FileReservation> files;
    std::size_t bytes = 0;
    std::string metadata_bytes;
};
// Exact reservations for this mapping slice. The caller must add the canonical
// source copies/chunks and all other static files before full PlanArchive.
MappingRecordPlan PlanMappingRecord(const PreparedSourceMapping&, const std::string& stem,
    std::size_t static_byte_cap = StaticReserveBytes);
RecordFile WriteMappingRecord(const std::filesystem::path&, const PreparedSourceMapping&,
    const std::string& stem, std::size_t static_byte_cap = StaticReserveBytes);
// This foundation needs already authenticated source bytes from CanonicalSource;
// it is not a self-contained source bundle or a completed simulation archive.
PreparedSourceMapping ReadMappingRecord(const std::filesystem::path&, const CanonicalSource&,
    const RecordFile&, const std::string& expected_mapping_sha256);
} // namespace crash::output::full_shell::source
