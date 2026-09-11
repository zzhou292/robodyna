#include "ReferenceStorage.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "lib_src/elements/qbat/QbatReference.h"
#include <stdexcept>

namespace crash::cases::vehicle_startup {
const char* Name(ReferenceStatus status) noexcept {
    switch(status) {
        case ReferenceStatus::UnresolvedDeclaration:return "unresolved_declaration";
        case ReferenceStatus::Success:return "kSuccess";
        case ReferenceStatus::InvalidInput:return "kInvalidInput";
        case ReferenceStatus::UnsupportedGeometry:return "kUnsupportedGeometry";
        case ReferenceStatus::NonfiniteResult:return "kNonfiniteResult";
        case ReferenceStatus::InvalidReference:return "kInvalidReference";
    }return "invalid_status";
}
namespace detail {
namespace {
template<class Status> ReferenceStatus Map(Status s) {
    switch(s) {
        case Status::kSuccess:return ReferenceStatus::Success;
        case Status::kInvalidInput:return ReferenceStatus::InvalidInput;
        case Status::kUnsupportedGeometry:return ReferenceStatus::UnsupportedGeometry;
        case Status::kNonfiniteResult:return ReferenceStatus::NonfiniteResult;
        case Status::kInvalidReference:return ReferenceStatus::InvalidReference;
    }
    throw std::runtime_error("Unrecognized native shell reference status");
}
void Finish(ReferenceStorage& out,ReferenceRow row) {
    ++out.counts.parents;++out.counts.status[static_cast<unsigned>(row.status)];
    if(row.status==ReferenceStatus::UnresolvedDeclaration)++out.counts.unresolved;
    else {
        ++out.counts.attempted;
        if(row.status==ReferenceStatus::Success)++out.counts.succeeded;
        else {++out.counts.rejected;if(out.first_error==SIZE_MAX)out.first_error=out.rows.size();}
    }
    out.rows.push_back(row);
}
}
void Append(ReferenceStorage& out,ReferenceRow row,const tl::fea::qeph::ReferenceInput& input) {
    tl::fea::qeph::ReferenceData next;
    row.family=ReferenceFamily::Qeph;row.reference_index=SIZE_MAX;
    row.status=Map(tl::fea::qeph::InitializeReference(input,next));++out.counts.qeph_attempted;
    if(row.status==ReferenceStatus::Success) {
        row.reference_index=out.qeph.size();out.qeph.push_back(next);++out.counts.qeph_succeeded;
    }
    Finish(out,row);
}
void Append(ReferenceStorage& out,ReferenceRow row,const tl::fea::t3::ReferenceInput& input) {
    tl::fea::t3::ReferenceData next;
    row.family=ReferenceFamily::T3;row.reference_index=SIZE_MAX;
    row.status=Map(tl::fea::t3::InitializeReference(input,next));++out.counts.t3_attempted;
    if(row.status==ReferenceStatus::Success) {
        row.reference_index=out.t3.size();out.t3.push_back(next);++out.counts.t3_succeeded;
    }
    Finish(out,row);
}
void Append(ReferenceStorage& out,ReferenceRow row,const tl::fea::qbat::ReferenceInput& input) {
    tl::fea::qbat::Reference next;
    row.family=ReferenceFamily::Qbat;
    row.reference_index=SIZE_MAX;
    row.status=Map(tl::fea::qbat::InitializeReference(input,next));
    ++out.counts.qbat_attempted;
    if(row.status==ReferenceStatus::Success) {
        row.reference_index=out.qbat.size();
        out.qbat.push_back(next);
        ++out.counts.qbat_succeeded;
    }
    Finish(out,row);
}
void AppendUnresolved(ReferenceStorage& out,ReferenceRow row) {
    row.family=ReferenceFamily::None;row.status=ReferenceStatus::UnresolvedDeclaration;row.reference_index=SIZE_MAX;
    Finish(out,row);
}
} // namespace detail
} // namespace crash::cases::vehicle_startup
