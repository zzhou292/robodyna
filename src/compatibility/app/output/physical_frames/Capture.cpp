#include "CaptureState.h"
#include "EnvironmentFields.h"
#include "case/vehicle_wall/native/EnvelopePhysicalSource.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::output::physical_frames {
void PhysicalAcceptedFrames::Capture(Run& run) {
    auto& s=*impl_;
    using Access=cases::vehicle_runtime::detail::CaptureAccess;
    const auto before=Access::Scope(run);
    detail::CheckBacking(s.mapping,run);
    detail::CheckIdentity(s.mapping,s.context,before);
    const auto stamp=Access::Nodes(run,s.positions.data(),s.velocities.data(),s.forecast.physical_nodes);
    Require(tl::fea::trial_identity::SameStamp(stamp,before.stamp),"Nodal accepted stamp changed during capture");
    auto& frame=s.frames.Staging();
    detail::StagePositions(s.mapping.physical_nodes(),s.positions.data(),s.forecast.physical_nodes,frame);
    const auto counts=s.mapping.physical_counts();
    const auto rendered=s.mapping.render_counts();
    detail::CheckReadback(before,Access::Qeph(run,stamp,s.layered.data(),s.flags.data(),counts.qeph));
    if(const auto* environment=s.mapping.environment()) {
        const auto& parent=environment->environment_parent();
        const detail::FixedEnvironmentField fixed{parent.qeph_index,parent.domain_nodes,environment->wall().geometry().reference_m};
        detail::CheckFixedEnvironment(fixed,s.positions.data(),s.velocities.data(),s.forecast.physical_nodes,
            s.layered.data(),s.flags.data(),counts.qeph);
    }
    // The complete actual QEPH family was read and its declared wall suffix
    // validated. Existing exact-count field staging receives the vehicle prefix.
    detail::StageLayered(s.context,s.mapping.parents(),QephFamily,s.layered.data(),s.flags.data(),rendered.qeph,
        frame,s.frames.flags);
    detail::CheckReadback(before,Access::T3(run,stamp,s.layered.data(),s.flags.data(),counts.t3));
    detail::StageLayered(s.context,s.mapping.parents(),T3Family,s.layered.data(),s.flags.data(),counts.t3,
        frame,s.frames.flags);
    detail::CheckReadback(before,Access::Qbat(run,stamp,s.qbat.data(),s.flags.data(),counts.qbat));
    detail::StageQbat(s.context,s.mapping.parents(),s.qbat.data(),s.flags.data(),counts.qbat,frame,s.frames.flags);
    const auto after=Access::Scope(run);
    detail::CheckSameEndpoint(before,after);
    s.frames.Finish(s.context,detail::Phase(after));
}
} // namespace crash::output::physical_frames
