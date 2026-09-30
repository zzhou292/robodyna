#include "Session.h"
#include "contact_diagnostics/Observe.h"
#include "output/ArtifactIO.h"
#include <chrono>
namespace crash::cases::vehicle_run {
Result PreparedRun::Execute(const std::filesystem::path& destination,const Control& control) const {
    return ExecuteImpl(destination,control,nullptr,nullptr);
}
Result PreparedRun::ExecuteImpl(const std::filesystem::path& destination,const Control& control,
                               void* initialization_context,Initialization initialize) const {
    output::Require(bool(initialization_context)==bool(initialize),
                    "Run initialization context and operation must be supplied together");
    output::Require(std::filesystem::symlink_status(destination).type()==std::filesystem::file_type::directory &&
        std::filesystem::is_empty(destination),"Run needs a real empty destination directory");
    detail::ValidateLoop(data_->horizon,data_->forecast.archive.archive.archive.frame_epochs,control);
    // This child owns the strict physical archive inventory. The bounded run
    // summary stays beside it and references its final manifest only if valid.
    std::filesystem::create_directory(destination/"archive");
    Result result;
    std::unique_ptr<Session> session;
    const auto startup_begin=std::chrono::steady_clock::now();
    const auto startup_elapsed=[&] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now()-startup_begin).count();
    };
    try {session=std::make_unique<Session>(*data_,destination/"archive",initialization_context,initialize);}
    catch(const std::exception& error) {
        result.startup_wall_s=startup_elapsed();
        result.loop.kind=StopKind::StartupFailure;
        result.loop.reason=std::string(error.what()).substr(0,4096);
        try {result.summary=detail::WriteSummary(destination,data_->config,data_->horizon,data_->forecast,result);}
        catch(const std::exception& e) {result.summary_error=std::string(e.what()).substr(0,4096);}
        return result;
    }
    result.startup_wall_s=startup_elapsed();
    result.session_initialized=true;
    if (const auto* contact = session->dynamics.self_contact_forecast())
        result.filter_initialization = contact->filter_initialization;
    const auto start=std::chrono::steady_clock::now();
    const auto clock=[&] {return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();};
    result.loop=detail::RunLoop(*session,data_->horizon,session->archive.forecast().archive.archive.frame_epochs,control,clock);
    result.mechanics_timing=session->dynamics.timing();
    result.last_contact_attempt=contact_diagnostics::Copy(session->dynamics.self_contact_diagnostics());
    result.archive_manifest=session->manifest;
    result.rejected_step_limit_s=session->step_limit;
    result.rejected_contact_status=session->contact_status;
    result.rejected_self_contact=session->self_contact_error;
    result.rejected_node=session->node;
    result.rejected_parent=session->parent;
    if(result.archive_manifest) {
        try {
            result.viewer_input=output::physical_run::WriteViewerInput(destination,"viewer-input.json",
                {"archive",*result.archive_manifest,data_->mapping.source_mapping().source().data().inputs,
                 data_->mapping.source_mapping().digest()});
        } catch(const std::exception& error) {
            result.viewer_input_error=std::string(error.what()).substr(0,4096);
        }
    }
    try {result.summary=detail::WriteSummary(destination,data_->config,data_->horizon,data_->forecast,result);}
    catch(const std::exception& error) {result.summary_error=std::string(error.what()).substr(0,4096);}
    return result;
}
} // namespace crash::cases::vehicle_run
