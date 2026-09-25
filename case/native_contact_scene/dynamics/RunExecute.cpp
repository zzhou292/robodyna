#include "RunInternal.h"
#include <chrono>
namespace crash::cases::native_scene {
namespace {
class Session final:public vehicle_run::detail::Operations {
  public:
    Session(const ContactSelection& source,const ArchiveSource& original,const RunConfig& config,
        output::full_shell::source::BundleRequest request,const std::filesystem::path& root)
      :dynamics(NativeSceneDynamics::Prepare(source,config.dynamics)),capture(dynamics.MakeCapture(original.mapping(),[&]{
        output::full_shell::Identity id;id.run=config.run_id;return id;}(),config.capture)),
        archive(output::physical_run::RunArchive::Prepare(root,original.mapping(),capture->context(),std::move(request),capture->ObservationProfile())){}
    NativeSceneDynamics dynamics;
    std::unique_ptr<output::physical_frames::NativeAcceptedFrames> capture;
    output::physical_run::RunArchive archive; // Destroy archive, capture, then physics.
    std::optional<output::full_shell::RecordFile> manifest;
    vehicle_run::Endpoint Accepted() const noexcept override {const auto s=dynamics.accepted();return {s.epoch,s.time};}
    void Prepare() override{dynamics.PrepareStep();}
    void Commit() override{dynamics.CommitStep();}
    void Discard() noexcept override{dynamics.DiscardStep();}
    void Append() override{archive.Append(capture->Interval());}
    void Capture() override{capture->Capture();}
    void SaveSample() override{
        output::Require(capture->frame()&&capture->activity(),"Native accepted frame/activity is unavailable");
        archive.Sample(*capture->frame(),*capture->activity());
    }
    void Finish(bool complete,const std::string& reason) override{manifest=complete?archive.Finish():archive.FinishPrefix(reason);}
};
}
RunResult PreparedNativeSceneRun::Execute(const std::filesystem::path& root,const vehicle_run::Control& control) const {
    using Clock=std::chrono::steady_clock;using Seconds=std::chrono::duration<double>;
    output::Require(std::filesystem::symlink_status(root).type()==std::filesystem::file_type::directory&&std::filesystem::is_empty(root),
        "Native run requires a real empty output directory");
    const auto& d=*data_;vehicle_run::detail::ValidateLoop(d.horizon,d.forecast.archive.archive.archive.frame_epochs,control);
    std::filesystem::create_directory(root/"archive");RunResult result;const auto start=Clock::now();std::unique_ptr<Session> session;
    try{session=std::make_unique<Session>(d.contact,d.archive_source,d.config,d.request,root/"archive");}
    catch(const std::exception& e){result.loop.kind=vehicle_run::StopKind::StartupFailure;result.loop.reason=std::string(e.what()).substr(0,4096);}
    result.startup_wall_s=Seconds(Clock::now()-start).count();
    if(session) {
        const auto begin=Clock::now();result.loop=vehicle_run::detail::RunLoop(*session,d.horizon,
            session->archive.forecast().archive.archive.frame_epochs,control,[&]{return Seconds(Clock::now()-begin).count();});
        result.manifest=session->manifest;
        if(result.manifest)try {
            result.viewer_input=output::physical_run::WriteViewerInput(root,"viewer-input.json",
                {"archive",*result.manifest,d.archive_source.mapping().source().data().inputs,d.archive_source.mapping().digest()});
        }catch(const std::exception& e){result.output_error=std::string(e.what()).substr(0,4096);}
    }
    try{result.summary=run_detail::Summary(root,d.config,d.horizon,d.forecast,d.contact,result);}
    catch(const std::exception& e){if(!result.output_error.empty())result.output_error+="; ";result.output_error+=std::string(e.what()).substr(0,4096);}
    return result;
}
}
