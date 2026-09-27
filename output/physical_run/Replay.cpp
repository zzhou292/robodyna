#include "Replay.h"
#include "output/BoundedArrayJson.h"
#include "ReplayBudget.h"
namespace crash::output::physical_run {
struct Replay::Data {
    Data(std::filesystem::path r,records::source::PreparedSourceMapping m,records::Context c,
        Configuration config,Index i,std::size_t cap):root(std::move(r)),mapping(std::move(m)),context(std::move(c)),
        configuration(std::move(config)),index(std::move(i)),host_bytes(cap) {}
    std::filesystem::path root;
    records::source::PreparedSourceMapping mapping;
    records::Context context;
    Configuration configuration;
    Index index;
    std::size_t host_bytes;
    std::optional<WallReceipt> wall;
    std::optional<EnvironmentReceipt> environment;
    std::optional<WallComposition> wall_composition;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh;
};
Replay Replay::Open(const std::filesystem::path& root,const records::RecordFile& file,
    const records::source::SourceInputs& source,const std::string& digest,ReplayLimits limits) {
    Require(limits.host_bytes && limits.host_bytes<=512u<<20 &&
        limits.source.host_bytes<=limits.host_bytes && (128u<<20)<=limits.host_bytes-limits.source.host_bytes,
        "Physical replay source and record workspace exceed host cap");
    const auto manifest=ReadManifest(array_json::Parse(ReadFile(root,file,MetadataCap),MetadataCap));
    Require(manifest.identity.source_mapping_sha256==digest,"Physical run mapping differs from caller authority");
    auto config=ReadConfiguration(array_json::Parse(ReadFile(root,manifest.configuration,MetadataCap),MetadataCap));
    Require(records::SameIdentity(manifest.identity,config.identity),"Physical configuration belongs to another run");
    Require(config.wall==bool(manifest.wall) && config.environment==bool(manifest.environment) &&
        !(manifest.wall&&manifest.environment),"Physical static environment profile/receipt presence differs");
    CheckInventory(root,file,manifest,config.request.total_byte_cap);
    auto mapping=records::source::ReadSourceBundle(root,manifest.source,source,digest,limits.source);
    auto context=mapping.MakeFrameContext(config.identity,config.request.fixed_dt,limits.records);
    const auto plan=records::activity::PlanWithActivity(context,config.request,"parent-activity.json");
    Require(plan.archive.forecast_bytes==manifest.forecast_bytes,"Physical whole-run forecast differs");
    const auto memory=replay_detail::Budget(context,config,limits.source.host_bytes,limits.host_bytes);
    auto index=ReadIndex(context,config,array_json::Parse(ReadFile(root,manifest.index,MetadataCap),MetadataCap));
    records::activity::ReadDeclaration(root,context,manifest.activity_declaration);
    ValidateRecords(root,context,config,index,128u<<20);
    CheckReferencedInventory(root,context,index,manifest);
    std::optional<WallComposition> composition;
    auto wall=manifest.wall?ReadWallArtifacts(root,*manifest.wall,mapping.source().data(),context,&composition):nullptr;
    if(manifest.environment)wall=ReadEnvironmentArtifacts(root,*manifest.environment,mapping.source().data(),context);
    if(manifest.wall)CheckWallBeamObservation(config.profile.beam18,composition);
    auto data=std::make_shared<Data>(root,std::move(mapping),std::move(context),std::move(config),std::move(index),memory.peak_host_bytes);
    data->wall=manifest.wall;data->environment=manifest.environment;data->wall_mesh=std::move(wall);
    data->wall_composition=composition;
    return Replay(std::move(data));
}
const records::source::PreparedSourceMapping& Replay::mapping() const noexcept {return data_->mapping;}
const records::Context& Replay::context() const noexcept {return data_->context;}
const Configuration& Replay::configuration() const noexcept {return data_->configuration;}
const Index& Replay::index() const noexcept {return data_->index;}
std::size_t Replay::peak_host_bytes() const noexcept {return data_->host_bytes;}
const WallReceipt* Replay::wall() const noexcept {return data_->wall?&*data_->wall:nullptr;}
const EnvironmentReceipt* Replay::environment() const noexcept {return data_->environment?&*data_->environment:nullptr;}
const WallComposition* Replay::wall_composition() const noexcept {
    return data_->wall_composition?&*data_->wall_composition:nullptr;
}
std::shared_ptr<const chrono::ChTriangleMeshConnected> Replay::wall_mesh() const noexcept {return data_->wall_mesh;}
Sample Replay::ReadSample(std::size_t k) const {
    Require(k<data_->index.frames.size(),"Physical replay sample index is outside the run");
    const auto& f=data_->index.frames[k];
    auto frame=records::ReadFrame(data_->root,data_->context,f.frame,f.stamp);
    auto activity=records::activity::ReadActivity(data_->root,data_->context,f.activity,f.stamp);
    return {std::move(frame),std::move(activity)};
}
} // namespace crash::output::physical_run
