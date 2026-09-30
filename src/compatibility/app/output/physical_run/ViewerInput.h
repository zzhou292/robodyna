#pragma once
#include "Replay.h"
namespace crash::output::physical_run {
inline constexpr std::size_t ViewerInputByteCap=64u<<10;
// An explicit caller-selected authority receipt, outside the strict archive.
// Source roots are deliberately absent: replay verifies the bundled source.
struct ViewerInput {
    std::string archive_directory;
    records::RecordFile manifest;
    records::source::SourceInputs source;
    std::string mapping_sha256;
};
Document ViewerInputDocument(const ViewerInput&);
ViewerInput ParseViewerInput(const Value&);
records::RecordFile WriteViewerInput(const std::filesystem::path& run_root,
    const std::string& filename,const ViewerInput&);
ViewerInput ReadViewerInput(const std::filesystem::path& run_root,
    const records::RecordFile& expected_descriptor);
std::filesystem::path ViewerArchivePath(const std::filesystem::path& run_root,const ViewerInput&);
} // namespace crash::output::physical_run
