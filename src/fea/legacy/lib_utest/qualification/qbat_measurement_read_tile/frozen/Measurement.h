// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "../QbatBatchMeasure.h"
#include "MeasurementValues.h"
namespace tl::fea::qbat::mapped {
struct CachedValidation {
  const AssemblyParent* values;
  TL_QBAT_HD bool operator()(std::size_t parent,const BatchResult&,const Material&,
      double,std::uint64_t) const noexcept { return values[parent].status==BatchStatus::Success; }
};
TL_QBAT_HD inline void MergeMaximum(MaximumSummary& a,const MaximumSummary& b) noexcept {
  a.valid=a.valid && b.valid;
  if(b.value>a.value) a.value=b.value;
}
TL_QBAT_HD inline bool FinishMeasurement(const batch_detail::Storage& state,
    const NodalPreparedView& view,BatchDiagnostics& output,unsigned blocks) noexcept {
  MaximumSummary maximum{0,true};
  for(unsigned block=0;block<blocks;++block) MergeMaximum(maximum,state.assembly.maximum[block]);
  if(maximum.valid) {
    if(maximum.value>output.maximum_displacement) output.maximum_displacement=maximum.value;
  } else if(!batch_detail::MeasureDisplacement(state.model,view,output)) {
    // Nonfinite source nodes retain the original rare failure-prefix diagnostic.
    return false;
  }
  return batch_detail::ValidMeasurement(output);
}
TL_QBAT_HD inline bool Measure(const batch_detail::Storage& state,const batch_detail::Slab& accepted,
    const batch_detail::Slab& trial,const NodalPreparedView& view,BatchDiagnostics& output,
    unsigned blocks) noexcept {
  // Retained direct caller: every signed parent/local accumulation is serial.
  return batch_detail::MeasureParents(state.model,accepted,trial,view,output,
      CachedValidation{state.assembly.parent}) && FinishMeasurement(state,view,output,blocks);
}
TL_QBAT_HD inline bool MeasureStaged(const batch_detail::Storage& state,
    const NodalPreparedView& view,BatchDiagnostics& output,unsigned blocks) noexcept {
  return MeasureStagedParents(state.model,state.assembly.measurement,output) &&
      FinishMeasurement(state,view,output,blocks);
}
TL_QBAT_HD inline void FinalizeMeasurement(batch_detail::Storage& state,
    const NodalPreparedView& view,BatchDiagnostics identity,unsigned blocks) noexcept {
  batch_detail::Control next{};
  next.diagnostics=identity;
  // Complete element-failure priority precedes every measurement failure,
  // including an invalid result at a lower source ordinal.
  for (std::size_t parent=0; parent<state.model.config.element_count; ++parent) {
    if (state.candidate_status[parent]==Status::kSuccess) continue;
    next.status=BatchStatus::ElementFailure;
    next.element=static_cast<std::uint32_t>(parent);
    next.element_status=state.candidate_status[parent];
    break;
  }
  if (next.status==BatchStatus::Success) {
    if (!MeasureStaged(state,view,next.diagnostics,blocks)) {
      next.status=BatchStatus::NonfiniteResult;
    } else {
      next.diagnostics.valid=true;
    }
  }
  state.control=next;
}
} // namespace tl::fea::qbat::mapped
