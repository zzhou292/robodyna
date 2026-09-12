#include "Session.h"
#include "ContactSummary.h"
#include "MechanicsSummary.h"
#include "SampledShellPlasticity.h"
namespace crash::cases::vehicle_run {
PreparedRun::Session::Session(const Data& input,const std::filesystem::path& directory)
    : source(input),dynamics(vehicle_wall::LoadedWall::Prepare(input.setup,input.dynamics,{},&input.joints)),
      capture(input.mapping,dynamics,input.identity),
      archive(output::physical_run::RunArchive::PrepareWithWall(directory,input.setup,input.mapping,
          capture.frames().context(),input.request,input.profile)) {}
Endpoint PreparedRun::Session::Accepted() const noexcept {
    const auto stamp=dynamics.accepted();
    return {stamp.epoch,stamp.time};
}
void PreparedRun::Session::Prepare() {
    try {dynamics.PrepareStep();}
    catch(const vehicle_dynamics::StepSizeError& error) {
        step_limit=error.limit();
        node=error.node();
        throw;
    } catch(const vehicle_wall::WallStageError& error) {
        contact_status=error.report().status;
        node=error.report().node;
        parent=error.report().parent;
        throw;
    }
}
void PreparedRun::Session::Commit() {dynamics.CommitStep();}
void PreparedRun::Session::Discard() noexcept {dynamics.DiscardStep();}
void PreparedRun::Session::Append() {
    const auto row=output::physical_run::CaptureAcceptedInterval(dynamics,capture.frames(),source.profile);
    auto next_contact = contact;
    auto next_mechanics = mechanics;
    ObserveAcceptedContact(next_contact, dynamics.last_accepted_step(), dynamics.accepted());
    ObserveAcceptedMechanics(next_mechanics, dynamics.last_accepted_step(), dynamics.accepted());
    archive.Append(row);
    contact = next_contact;
    mechanics = next_mechanics;
}
void PreparedRun::Session::Capture() {capture.Capture(dynamics);}
void PreparedRun::Session::SaveSample() {
    const auto& frames = capture.frames();
    const auto* frame = frames.frame();
    const auto* activity = frames.activity();
    output::Require(frame && activity, "Saved shell summary requires an available accepted frame/activity pair");
    detail::SaveShellSample(sampled_shell_plasticity, frames.context(), *frame,
        [&] { archive.Sample(*frame, *activity); });
}
void PreparedRun::Session::Finish(bool complete,const std::string& reason) {
    manifest=complete?archive.Finish():archive.FinishPrefix(reason);
}
} // namespace crash::cases::vehicle_run
