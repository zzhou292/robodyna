#include "CaptureAccess.h"
#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime::detail {
AcceptedCaptureScope CaptureAccess::Scope(VehiclePhysicalStartup& run) {
    output::Require(bool(run.storage_),"Accepted capture owner handle is empty");
    auto& s=*run.storage_;
    AcceptedCaptureScope out;
    out.stamp=s.owner.accepted();
    RequireSuccess(s.publication.CopyAcceptedPhysicalDiagnostics(out.stamp,&out.diagnostics));
    output::Require(bool(s.type45)==bool(s.joint_model) && out.diagnostics.has_type45==bool(s.type45),
        "Accepted joint participant differs from actual retained startup");
    if(s.type45) {
        out.type45_source_instance_id=s.joint_model->model().source_instance_id();
        out.type45_joint_count=s.joint_model->model().joints().size();
        tl::fea::type45::BatchDiagnostics actual;
        RequireSuccess(s.type45->CopyAcceptedDiagnostics(out.stamp,&actual));
        const auto& common=out.diagnostics.type45;
        output::Require(actual.valid && actual.owner_id==common.owner_id && actual.epoch==common.epoch &&
            actual.attempt==common.attempt && actual.base_epoch==common.base_epoch && actual.phase==common.phase &&
            actual.source_instance_id==out.type45_source_instance_id && actual.joint_count==out.type45_joint_count &&
            actual.configuration_id==common.configuration_id && actual.qualification_id==common.qualification_id &&
            output::Bits(actual.time)==output::Bits(common.time) &&
            output::Bits(actual.velocity_time)==output::Bits(common.velocity_time) &&
            output::Bits(actual.base_time)==output::Bits(common.base_time) &&
            output::Bits(actual.kick_dt)==output::Bits(common.kick_dt) &&
            actual.automatic_stiffness_initialized==common.automatic_stiffness_initialized,
            "Accepted joint publication differs from actual batch");
    }
    return out;
}
tl::fea::NodalStamp CaptureAccess::Nodes(VehiclePhysicalStartup& run,double* x,double* v,std::size_t count) {
    output::Require(bool(run.storage_) && count==run.storage_->owner.accepted().node_count,
                    "Accepted nodal capture requires the complete physical domain");
    tl::fea::NodalStamp stamp;
    RequireSuccess(run.storage_->owner.CopyAccepted({x,v,count},&stamp));
    return stamp;
}
tl::fea::qeph::BatchDiagnostics CaptureAccess::Qeph(VehiclePhysicalStartup& run,const tl::fea::NodalStamp& stamp,
    tl::fea::ShellBatchLayeredSection* history,std::uint8_t* active,std::size_t count) {
    output::Require(bool(run.storage_),"Accepted capture owner handle is empty");
    tl::fea::qeph::BatchDiagnostics out;
    RequireSuccess(run.storage_->qeph.CopyAcceptedLayeredSectionHistory(stamp,history,count,&out));
    RequireSuccess(run.storage_->qeph.CopyAcceptedParentActivity(stamp,active,count,&out));
    return out;
}
tl::fea::t3::BatchDiagnostics CaptureAccess::T3(VehiclePhysicalStartup& run,const tl::fea::NodalStamp& stamp,
    tl::fea::ShellBatchLayeredSection* history,std::uint8_t* active,std::size_t count) {
    output::Require(bool(run.storage_),"Accepted capture owner handle is empty");
    tl::fea::t3::BatchDiagnostics out;
    RequireSuccess(run.storage_->t3.CopyAcceptedLayeredSectionHistory(stamp,history,count,&out));
    RequireSuccess(run.storage_->t3.CopyAcceptedParentActivity(stamp,active,count,&out));
    return out;
}
tl::fea::qbat::BatchDiagnostics CaptureAccess::Qbat(VehiclePhysicalStartup& run,const tl::fea::NodalStamp& stamp,
    tl::fea::qbat::BatchResult* history,std::uint8_t* active,std::size_t count) {
    output::Require(bool(run.storage_),"Accepted capture owner handle is empty");
    tl::fea::qbat::BatchDiagnostics out;
    RequireSuccess(run.storage_->qbat.CopyAcceptedResults(stamp,history,count,&out));
    RequireSuccess(run.storage_->qbat.CopyAcceptedParentActivity(stamp,active,count,&out));
    return out;
}
} // namespace crash::cases::vehicle_runtime::detail
