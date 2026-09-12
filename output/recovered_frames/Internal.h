#pragma once
#include "Replay.h"
#include "output/BoundedArrayJson.h"

namespace crash::output::recovered_frames {
namespace records = full_shell;
namespace run = physical_run;
struct Description {
    records::source::SourceInputs source;
    std::string mapping_sha256,reason;
    records::RecordFile configuration,source_bundle,activity_declaration;
    std::optional<run::WallReceipt> wall;
    std::vector<run::FrameFiles> frames;
    std::vector<records::RecordFile> files;
};
Document Encode(const Description&);
Description Decode(const Value&);
struct LoadedSource {
    records::source::PreparedSourceMapping mapping;
    records::Context context;
    run::Configuration configuration;
    std::size_t peak_host_bytes;
};
LoadedSource LoadSource(const std::filesystem::path&,const Description&,Limits);
records::RecordFile InspectFile(const std::filesystem::path&,const std::string&,std::size_t);
void CheckFrames(const records::Context&,const run::Configuration&,const std::vector<run::FrameFiles>&);
std::vector<run::FrameFiles> ScanFrames(const std::filesystem::path&,const records::Context&,const run::Configuration&);
std::vector<records::RecordFile> ReferencedFiles(const std::filesystem::path&,const records::Context&,const Description&);
void CheckInventory(const std::filesystem::path&,const Description&,std::size_t cap,bool closed);
std::size_t Budget(const records::Context&,const run::Configuration&,Limits);
void CopyFiles(const std::filesystem::path&,const std::filesystem::path&,
    const std::vector<records::RecordFile>&,std::size_t cap);
} // namespace crash::output::recovered_frames
