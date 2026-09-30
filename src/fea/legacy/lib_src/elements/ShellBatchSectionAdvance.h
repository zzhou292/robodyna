#pragma once
#include "ShellBatchPlasticity.h"
#if defined(__CUDACC__)
#define TL_SECTION_ADVANCE_HD __host__ __device__
#else
#define TL_SECTION_ADVANCE_HD
#endif
namespace tl::fea::shell_batch_plasticity_detail {
// Same native work-channel expression/order as the original resident LAW44
// path. This channel is observational and never added to shell work again.
TL_SECTION_ADVANCE_HD inline bool ProposedPlasticSection(const ShellBatchSectionState& old,
    const sections::ShellLayeredJ2History& history,const sections::ShellLayeredJ2Diagnostics& diagnostics,
    double force_thickness,double current_area,ShellBatchSectionState& output) noexcept {
  ShellBatchSectionState next;next.history=history;next.diagnostics=diagnostics;
  next.cumulative_plastic_work_J=old.cumulative_plastic_work_J+
    diagnostics.plastic_work_density_increment*force_thickness*current_area;
  if(!tl::math::Finite(next.cumulative_plastic_work_J))return false;
  output=next;return true;
}
} // namespace tl::fea::shell_batch_plasticity_detail
#undef TL_SECTION_ADVANCE_HD
