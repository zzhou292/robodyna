#pragma once
#include "Types.h"
#include "WallArtifacts.h"
#include "output/full_shell/FullShellVisualizationPlan.h"
namespace crash::output::physical_run {
struct Configuration {
    records::Identity identity;
    Profile profile;
    records::PlanRequest request;
    std::string point_layout_sha256;
    bool wall=false;
};
Document ConfigurationDocument(const Configuration&);
Configuration ReadConfiguration(const Value&);
Document IndexDocument(const Configuration&,const Index&);
Index ReadIndex(const records::Context&,const Configuration&,const Value&);
void CheckIndex(const records::Context&,const Configuration&,const Index&);
struct Manifest {
    records::Identity identity;
    records::RecordFile configuration,index,source,activity_declaration;
    std::vector<records::RecordFile> inventory;
    std::size_t forecast_bytes=0;
    std::optional<WallReceipt> wall;
};
Document ManifestDocument(const Manifest&);
Manifest ReadManifest(const Value&);
std::vector<records::RecordFile> Inventory(const std::filesystem::path&,std::size_t total_cap,bool wall=false);
void CheckInventory(const std::filesystem::path&,const records::RecordFile& manifest,const Manifest&,std::size_t total_cap);
// Called after source and record validation. Every inventoried file must have a
// typed owner in those authenticated descriptors; arbitrary companions reject.
void CheckReferencedInventory(const std::filesystem::path&,const records::Context&,const Index&,const Manifest&);
// Exact typed owners shared by closed-run and explicitly recovered sample
// readers. Configuration, index, interval streams and outer descriptors are
// deliberately not included; their enclosing schema owns those obligations.
std::vector<records::RecordFile> ReferencedSampleFiles(const std::filesystem::path&,
    const records::Context&,const records::RecordFile& source_bundle,
    const records::RecordFile& activity_declaration,const std::vector<FrameFiles>&,
    const std::optional<WallReceipt>&);
} // namespace crash::output::physical_run
