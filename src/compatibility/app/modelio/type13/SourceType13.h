#pragma once
#include "ConvertProperty.h"
#include "modelio/source_assembly/SourceAssemblyData.h"
#include <filesystem>
#include <memory>
#include <vector>

namespace crash::modelio::type13 {
using ArtifactIdentity=assembly::ArtifactIdentity;
struct Node {
    std::uint64_t id=0;std::size_t source_line=0;unsigned blank_mask=0;
    native::Vec3 position_native{},position_m{};std::string raw_text;
};
struct Beam {
    std::uint64_t id=0;std::size_t source_line=0,canonical_index=0,node_indices[3]{};
    std::string raw_text;native::Startup startup{};
};
struct Data {
    ArtifactIdentity identity{};
    std::size_t startup_budget_bytes=0,owned_payload_bytes=0;
    std::string authenticated_bytes,canonical_manifest_sha256;
    assembly::SourceBlock part_source,section_source,material_source;
    SourceProperty source_property{};ConvertedProperty converted{};native::Property property{};
    std::vector<Node> nodes;std::vector<Beam> beams;
};
struct ReadLimits {
    std::size_t bytes=8*1024*1024,nodes=8192,beams=8192;
    std::size_t host_bytes=128*1024*1024;
};
class SourceType13 {
 public:
    static SourceType13 Read(const std::filesystem::path&,const ArtifactIdentity&,ReadLimits={});
    static SourceType13 ReadBytes(const std::string&,const ArtifactIdentity&,ReadLimits={});
    const Data& data() const;
 private:
    explicit SourceType13(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
}
