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
