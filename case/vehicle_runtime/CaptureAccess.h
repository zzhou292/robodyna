#pragma once
#include "VehiclePhysicalStartup.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
namespace crash::cases::vehicle_runtime::detail {
struct AcceptedCaptureScope {
    tl::fea::NodalStamp stamp;
    tl::fea::ShellPhysicalDiagnostics diagnostics;
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
