#pragma once
#include "FieldTypes.h"
#include "output/full_shell/FullShellVisualizationRecords.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/qbat/QbatBatchTypes.h"
namespace crash::output::physical_frames::detail {
// Formatting into unpublished caller scratch only. These helpers create no
// owner/source/acceptance authority and never change native values.
void StagePositions(const std::vector<std::uint32_t>&,const double*,std::size_t,records::FrameRecord&);
void StageLayered(const records::Context&,const std::vector<ParentField>&,std::uint32_t,
    const tl::fea::ShellBatchLayeredSection*,const std::uint8_t*,std::size_t,
    records::FrameRecord&,std::vector<std::uint8_t>&);
void StageQbat(const records::Context&,const std::vector<ParentField>&,
    const tl::fea::qbat::BatchResult*,const std::uint8_t*,std::size_t,
    records::FrameRecord&,std::vector<std::uint8_t>&);
}
