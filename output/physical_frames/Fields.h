#pragma once
#include "Mapping.h"
#include "case/vehicle_runtime/CaptureAccess.h"
namespace crash::output::physical_frames::detail {
using CaptureScope=cases::vehicle_runtime::detail::AcceptedCaptureScope;
records::FrameStamp Phase(const CaptureScope&);
void CheckReadback(const CaptureScope&,const tl::fea::qeph::BatchDiagnostics&);
void CheckReadback(const CaptureScope&,const tl::fea::t3::BatchDiagnostics&);
void CheckReadback(const CaptureScope&,const tl::fea::qbat::BatchDiagnostics&);
void CheckIdentity(const Mapping&,const records::Context&,const CaptureScope&);
void CheckSameEndpoint(const CaptureScope&,const CaptureScope&);
void StagePositions(const std::vector<std::uint32_t>& physical_nodes,const double* xyz,std::size_t physical_count,
                    records::FrameRecord&);
// These functions write only a caller's unpublished frame slot. A failure can
// leave that private slot incomplete; the public producer selects it atomically.
void StageLayered(const records::Context&,const std::vector<ParentField>&,std::uint32_t family,
    const tl::fea::ShellBatchLayeredSection*,const std::uint8_t*,std::size_t count,
    records::FrameRecord&,std::vector<std::uint8_t>& activity);
void StageQbat(const records::Context&,const std::vector<ParentField>&,
    const tl::fea::qbat::BatchResult*,const std::uint8_t*,std::size_t count,
    records::FrameRecord&,std::vector<std::uint8_t>& activity);
} // namespace crash::output::physical_frames::detail
