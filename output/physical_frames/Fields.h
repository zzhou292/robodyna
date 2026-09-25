#pragma once
#include "Mapping.h"
#include "RecordFields.h"
#include "case/vehicle_runtime/CaptureAccess.h"
namespace crash::output::physical_frames::detail {
using CaptureScope=cases::vehicle_runtime::detail::AcceptedCaptureScope;
records::FrameStamp Phase(const CaptureScope&);
void CheckReadback(const CaptureScope&,const tl::fea::qeph::BatchDiagnostics&);
void CheckReadback(const CaptureScope&,const tl::fea::t3::BatchDiagnostics&);
void CheckReadback(const CaptureScope&,const tl::fea::qbat::BatchDiagnostics&);
void CheckIdentity(const Mapping&,const records::Context&,const CaptureScope&);
void CheckSameEndpoint(const CaptureScope&,const CaptureScope&);
} // namespace crash::output::physical_frames::detail
