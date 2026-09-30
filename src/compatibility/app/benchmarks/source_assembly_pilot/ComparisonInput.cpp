#include "ComparisonInput.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <stdexcept>

namespace crash::benchmarks::assembly_pilot {
std::string Encode(const Value& value) {
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    Require(value.Accept(writer),"Cannot serialize pilot report");return {buffer.GetString(),buffer.GetSize()};
}
Document PhysicalConfiguration(const Value& config) {
    Require(config.IsObject(),"Pilot configuration must be an object");
    Document next;next.CopyFrom(config,next.GetAllocator());
    // Only explicit time/output identities, resource ceilings and the optional
    // diagnostic switch vary. Keep all present/future physical declarations.
    for(const char* name:{"owner_id","run_id","topology_id","fixed_dt_s","requested_steps","frame_every",
        "archive_byte_cap","artifact_file_byte_cap","frame_cap","forecast_bytes","forecast_frames","forecast_files",
        "interval_ledger_segments","storage_limits","observe_force_stage"})next.RemoveMember(name);
    return next;
}
void Run::Open(const std::filesystem::path& path) {
    const auto manifest=output::ReadBounded(path/"manifest.json",1024*1024);
    manifest_hash=output::Sha256(manifest);
    // Reuse the owning reader for every frame/source/phase/ledger check. It
    // retains only one frame and requires unchanged files while being read.
    output::AcceptedReplay reader;const auto report=reader.Open(path);
    if(report.status!=output::ReplayStatus::Ok)throw std::runtime_error(report.diagnostic);
    Require(reader.info()->kind==output::ReplayKind::SourceAssemblyWall,"Pilot requires source_assembly_wall archives");
    Require(reader.info()->source_assembly&&reader.info()->source_assembly->inventory_sha256==
            modelio::assembly::PinnedYarisSixPartInventory().sha256,
            "Pilot comparison supports only the pinned six-part component without connectors; seven-part connector comparison is not qualified");
    bundle=rd::ReadIndex(path);
    Require(output::Sha256(output::ReadBounded(path/"manifest.json",1024*1024))==manifest_hash,
            "Pilot manifest changed during validation");
    configuration=rd::Json(rd::VerifiedBytes(bundle,"configuration.json"));physical=PhysicalConfiguration(configuration);
    placement=rd::VerifiedBytes(bundle,"placed-wall-placement.json");placed_mesh=rd::VerifiedBytes(bundle,"placed-wall.mesh.json");
    original_wall=rd::VerifiedBytes(bundle,"original-canonical-wall.manifest.json");
}
std::vector<std::uint64_t> Run::Epochs() const {
    std::vector<std::uint64_t> result;result.reserve(bundle.entries.size());
    for(const auto& e:bundle.entries)result.push_back(e.epoch);return result;
}
Document Run::Frame(std::size_t index) const {
    Require(index<bundle.entries.size(),"Pilot frame index is outside its accepted inventory");
    const auto& e=bundle.entries[index];
    auto next=rd::Json(rd::VerifiedBytes(bundle,e.mesh.substr(0,e.mesh.size()-10)+".fields.json"));
    Require(rd::Unsigned(next,"accepted_epoch")==e.epoch&&
            output::Bits(rd::Real(next,"accepted_time_s"))==output::Bits(e.time),"Pilot frame association changed");
    return next;
}
void MatchRuns(const Run& a,const Run& b) {
    Require(a.physical==b.physical&&a.placement==b.placement&&a.placed_mesh==b.placed_mesh&&a.original_wall==b.original_wall,
            "Pilot changed source/material/native M/J/group/boundary/wall/physical-envelope declarations");
    Require(Tick(a.bundle.assembly->requested_steps,a.multiple)==Tick(b.bundle.assembly->requested_steps,b.multiple),
            "Pilot requested physical horizons differ");
}
} // namespace crash::benchmarks::assembly_pilot
