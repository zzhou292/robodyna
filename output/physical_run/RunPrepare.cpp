#include "RunState.h"
#include "lib_utils/BoundedArena.h"
namespace crash::output::physical_run {
records::source::BundleRequest MakeRequest(const records::Context& c,std::uint64_t intervals,
    double duration,std::size_t samples,std::size_t total_cap) {
    records::source::BundleRequest request;
    auto& r=request.archive;
    r.nodes=c.nodes();r.parents=c.parents().size();r.plastic_points=c.points();
    r.intervals=intervals;r.requested_duration=duration;r.frames=samples;r.fixed_dt=c.fixed_dt();
    r.total_byte_cap=total_cap;
    r.static_files={{"manifest.json",MetadataCap},{"frame-index.json",MetadataCap},{"configuration.json",MetadataCap}};
    records::activity::PlanWithActivity(c,r,"parent-activity.json");return request;
}
namespace detail {
void ValidateRequest(const records::PlanRequest& r,Profile p,bool wall) {
    Require(r.extra_interval_bytes==ExtraIntervalBytes(p) && !r.extra_frame_bytes,
        "Physical run optional storage differs from its observation profile");
    Require(r.static_files.size()==(wall?10u:3u),"Physical run static reservation count differs from named profile");
    for(const auto* name:{"manifest.json","frame-index.json","configuration.json"}) {
        bool found=false;
        for(const auto& f:r.static_files)if(f.file==name)found=f.bytes==MetadataCap;
        Require(found,"Physical run metadata reservation differs from writer cap");
    }
    if(wall)for(const auto* name:WallFiles) {
        bool found=false;
        for(const auto& f:r.static_files)if(f.file==name)found=f.bytes==WallFileCap;
        Require(found,"Physical wall reservation differs from fixed artifact cap");
    }
}
Forecast ForecastRun(const records::Context& c,const physical_frames::Archive& a,Profile p,Limits limits,bool wall) {
    Require(limits.host_bytes && limits.host_bytes<=512u<<20,"Invalid physical run host cap");
    Forecast f;f.archive=a.plan();
    const auto rows=std::min<std::uint64_t>(f.archive.archive.rows_per_chunk,
        f.archive.archive.interval_bytes/(interval::RowBytes+ExtraIntervalBytes(p)));
    f.interval_staging_bytes=rows*8*(IntegerFields(p).size()+RealFields(p).size());
    tl::util::BoundedArenaLayout budget(limits.host_bytes);tl::util::ArenaRegion region;
    Require(budget.Append<std::byte>(a.startup_host_bytes(),region) &&
        budget.Append<std::byte>(3*f.interval_staging_bytes,region) &&
        budget.Append<std::byte>(32*MetadataCap,region) &&
        budget.Append<std::byte>(sizeof(FrameFiles)*f.archive.archive.frame_capacity,region) &&
        budget.Append<std::byte>(wall?WallWorkspaceBytes:0,region),
        "Physical run source/frame/interval/metadata peak exceeds host cap");
    f.peak_host_bytes=budget.bytes();return f;
}
}
Forecast RunArchive::Preflight(const records::source::PreparedSourceMapping& mapping,const records::Context& context,
    records::source::BundleRequest request,Profile profile,Limits limits) {
    return PreflightCore(mapping,context,std::move(request),profile,limits,false);
}
Forecast RunArchive::PreflightCore(const records::source::PreparedSourceMapping& mapping,const records::Context& context,
    records::source::BundleRequest request,Profile profile,Limits limits,bool wall) {
    if(!request.archive.extra_interval_bytes)request.archive.extra_interval_bytes=ExtraIntervalBytes(profile);
    detail::ValidateRequest(request.archive,profile,wall);
    auto frame_archive=physical_frames::Archive::Prepare(mapping,context,std::move(request));
    return detail::ForecastRun(context,frame_archive,profile,limits,wall);
}
RunArchive RunArchive::Prepare(const std::filesystem::path& root,const records::source::PreparedSourceMapping& mapping,
    const records::Context& context,records::source::BundleRequest request,Profile profile,Limits limits) {
    return PrepareCore(root,mapping,context,std::move(request),profile,limits,false);
}
RunArchive RunArchive::PrepareCore(const std::filesystem::path& root,const records::source::PreparedSourceMapping& mapping,
    const records::Context& context,records::source::BundleRequest request,Profile profile,Limits limits,bool wall) {
    if(!request.archive.extra_interval_bytes)request.archive.extra_interval_bytes=ExtraIntervalBytes(profile);
    detail::ValidateRequest(request.archive,profile,wall);
    auto frame_archive=physical_frames::Archive::Prepare(mapping,context,request);
    const auto forecast=detail::ForecastRun(context,frame_archive,profile,limits,wall);
    for(const auto& reserve:frame_archive.source_bundle().reservations())request.archive.static_files.push_back(reserve);
    Configuration config{context.identity(),profile,request.archive,context.point_layout_sha256(),wall};
    ConfigurationDocument(config);
    Require(std::filesystem::symlink_status(root).type()==std::filesystem::file_type::directory &&
        std::filesystem::is_empty(root),"Physical run needs a real empty destination directory");
    auto state=std::make_unique<Data>(root,context,std::move(frame_archive),std::move(config),forecast);
    state->index.planned_intervals=request.archive.intervals;
    state->index.frames.reserve(forecast.archive.archive.frame_capacity);
    state->intervals=std::make_unique<IntervalWriter>(root,context,profile,request.archive.intervals,
        request.archive.file_byte_cap,limits.host_bytes);
    // Every allocation/preflight above precedes the first output mutation.
    std::filesystem::create_directory(root/"arrays");
    state->manifest.identity=context.identity();state->manifest.forecast_bytes=forecast.archive.archive.forecast_bytes;
    state->manifest.source=records::source::WriteSourceBundle(root,state->frames.source_bundle());
    state->manifest.activity_declaration=records::activity::WriteDeclaration(root,"parent-activity.json",context);
    state->manifest.configuration=WriteDocument(root,"configuration.json",ConfigurationDocument(state->configuration),MetadataCap);
    return RunArchive(std::move(state));
}
RunArchive::RunArchive(std::unique_ptr<Data> data):data_(std::move(data)) {}
RunArchive::~RunArchive()=default;
RunArchive::RunArchive(RunArchive&&) noexcept=default;
RunArchive& RunArchive::operator=(RunArchive&&) noexcept=default;
const Forecast& RunArchive::forecast() const noexcept {return data_->forecast;}
std::uint64_t RunArchive::accepted_intervals() const noexcept {return data_->intervals->sequence().last.epoch;}
bool RunArchive::failed() const noexcept {return data_->failed || data_->intervals->failed();}
} // namespace crash::output::physical_run
