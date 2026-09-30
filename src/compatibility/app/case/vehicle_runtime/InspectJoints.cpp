#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
namespace {
bool Zero(tl::math::Vec3 x) noexcept { return x.x==0 && x.y==0 && x.z==0; }
}
void VehiclePhysicalStartup::Storage::InspectJoints(InitialInspection& output) {
    if(!type45) return;
    const auto joints=source.joints()->joints();
    std::vector<tl::fea::type45::Result> rows(joints.size());
    tl::fea::type45::BatchDiagnostics diagnostics;
    detail::RequireSuccess(type45->CopyAcceptedResults(owner.accepted(),{rows.data(),rows.size()},&diagnostics));
    output::Require(diagnostics.valid && diagnostics.phase==tl::fea::type45::BatchPhase::Accepted &&
        diagnostics.epoch==0 && diagnostics.time==0 && diagnostics.joint_count==joints.size() &&
        !diagnostics.automatic_stiffness_initialized && !diagnostics.has_completed_interval,
        "Initial joints unexpectedly claim an interval or automatic stiffness");
    for(std::size_t i=0;i<rows.size();++i) {
        const auto& row=rows[i];
        output::Require(row.source_joint_id==joints[i].geometry.source_joint_id &&
            row.stamp.sample_index==0 && row.stamp.time_s==0 && !row.automatic_stiffness_initialized &&
            Zero(row.history.local_force_n) && Zero(row.history.local_couple_nm),
            "Initial joint source/history differs from its virgin cache");
        for(const auto& endpoint:row.endpoint)
            output::Require(Zero(endpoint.force_n) && Zero(endpoint.couple_nm) &&
                endpoint.translational_stiffness_n_m==0 && endpoint.rotational_stiffness_nm==0,
                "Initial joint cache must precede native automatic stiffness and carry no load");
    }
    output.type45_joints=rows.size();
}
} // namespace crash::cases::vehicle_runtime
