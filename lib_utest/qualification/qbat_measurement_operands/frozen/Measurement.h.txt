// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "../QbatBatchMeasure.h"
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
TL_QBAT_HD inline bool Measure(const batch_detail::Storage& state,const batch_detail::Slab& accepted,
    const batch_detail::Slab& trial,const NodalPreparedView& view,BatchDiagnostics& output,
    unsigned blocks) noexcept {
  // Keep every signed parent/local accumulation and its failure prefix serial.
  if(!batch_detail::MeasureParents(state.model,accepted,trial,view,output,
      CachedValidation{state.assembly.parent})) return false;
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
} // namespace tl::fea::qbat::mapped
