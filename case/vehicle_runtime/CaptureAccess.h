#pragma once
#include "VehiclePhysicalStartup.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
namespace crash::cases::vehicle_runtime::detail {
struct AcceptedCaptureScope {
    tl::fea::NodalStamp stamp;
    tl::fea::ShellPhysicalDiagnostics diagnostics;
    // Read from the retained immutable joint model and actual optional batch.
    std::uint64_t type45_source_instance_id=0;
    std::size_t type45_joint_count=0;
    std::uint64_t beam18_source_instance_id=0;
    std::size_t beam18_parent_count=0;
};
// Internal read-only bridge for the full physical accepted output adapter. It
// exports no mutable owner, participant, stream, trial token or storage pointer.
struct CaptureAccess {
    static AcceptedCaptureScope Scope(VehiclePhysicalStartup&);
    static tl::fea::NodalStamp Nodes(VehiclePhysicalStartup&,double* xyz,double* velocity,std::size_t count);
    static tl::fea::qeph::BatchDiagnostics Qeph(VehiclePhysicalStartup&,const tl::fea::NodalStamp&,
        tl::fea::ShellBatchLayeredSection*,std::uint8_t* activity,std::size_t count);
    static tl::fea::t3::BatchDiagnostics T3(VehiclePhysicalStartup&,const tl::fea::NodalStamp&,
        tl::fea::ShellBatchLayeredSection*,std::uint8_t* activity,std::size_t count);
    static tl::fea::qbat::BatchDiagnostics Qbat(VehiclePhysicalStartup&,const tl::fea::NodalStamp&,
        tl::fea::qbat::BatchResult*,std::uint8_t* activity,std::size_t count);
};
} // namespace crash::cases::vehicle_runtime::detail
