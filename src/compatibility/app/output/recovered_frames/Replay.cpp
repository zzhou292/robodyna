#include "Internal.h"

namespace crash::output::recovered_frames {
struct Replay::Data {
    Data(std::filesystem::path r,LoadedSource s,Description d,records::RecordFile f)
        :root(std::move(r)),source(std::move(s)),description(std::move(d)),file(std::move(f)) {}
    std::filesystem::path root;
    LoadedSource source;
    Description description;
    records::RecordFile file;
    std::optional<run::WallComposition> composition;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh;
};
Replay Replay::Open(const std::filesystem::path& root,const records::RecordFile& expected,Limits limits) {
    Require(expected.file==DescriptorFilename,"Expected the distinct recovered-sample descriptor");
    auto description=Decode(array_json::Parse(run::ReadFile(root,expected,run::MetadataCap),run::MetadataCap));
    auto source=LoadSource(root,description,limits);
    CheckInventory(root,description,source.configuration.request.total_byte_cap,true);
    CheckFrames(source.context,source.configuration,description.frames);
    for(const auto& frame:description.frames) {
        records::ReadFrame(root,source.context,frame.frame,frame.stamp);
        records::activity::ReadActivity(root,source.context,frame.activity,frame.stamp);
    }
    const auto owners=ReferencedFiles(root,source.context,description);
    Require(owners.size()==description.files.size(),"Recovered inventory lacks exact typed owners");
    for(std::size_t i=0;i<owners.size();++i)Require(owners[i].file==description.files[i].file &&
        owners[i].sha256==description.files[i].sha256 && owners[i].bytes==description.files[i].bytes,
        "Recovered inventory differs from typed payload references");
    auto data=std::make_shared<Data>(root,std::move(source),std::move(description),expected);
    if(data->description.wall) {
        data->wall_mesh=run::ReadWallArtifacts(root,*data->description.wall,data->source.mapping.source().data(),
            data->source.context,&data->composition);
        run::CheckWallBeamObservation(data->source.configuration.profile.beam18,data->composition);
    }
    if(data->description.environment)
        data->wall_mesh=run::ReadEnvironmentArtifacts(root,*data->description.environment,
            data->source.mapping.source().data(),data->source.context);
    return Replay(std::move(data));
}
const records::source::PreparedSourceMapping& Replay::mapping() const noexcept {return data_->source.mapping;}
const records::Context& Replay::context() const noexcept {return data_->source.context;}
const run::Configuration& Replay::configuration() const noexcept {return data_->source.configuration;}
std::size_t Replay::peak_host_bytes() const noexcept {return data_->source.peak_host_bytes;}
const std::vector<run::FrameFiles>& Replay::frames() const noexcept {return data_->description.frames;}
run::Sample Replay::ReadSample(std::size_t i) const {
    Require(i<frames().size(),"Recovered sample index is outside retained frames");
    const auto& frame=frames()[i];
    return {records::ReadFrame(data_->root,context(),frame.frame,frame.stamp),
        records::activity::ReadActivity(data_->root,context(),frame.activity,frame.stamp)};
}
const run::WallReceipt* Replay::wall() const noexcept {return data_->description.wall?&*data_->description.wall:nullptr;}
const run::EnvironmentReceipt* Replay::environment() const noexcept {return data_->description.environment?&*data_->description.environment:nullptr;}
const run::WallComposition* Replay::wall_composition() const noexcept {return data_->composition?&*data_->composition:nullptr;}
std::shared_ptr<const chrono::ChTriangleMeshConnected> Replay::wall_mesh() const noexcept {return data_->wall_mesh;}
const std::string& Replay::stop_reason() const noexcept {return data_->description.reason;}
const records::RecordFile& Replay::descriptor() const noexcept {return data_->file;}
} // namespace crash::output::recovered_frames
