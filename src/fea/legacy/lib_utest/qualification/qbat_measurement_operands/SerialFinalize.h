// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_mapped_gather/SerialMeasure.h"
namespace qbat_measurement_test::serial {
using namespace tl::fea;
using namespace tl::fea::qbat;
using namespace tl::fea::qbat::batch_detail;
inline void Finalize(Storage* storage,const Slab* accepted,const Slab* trial,
    NodalPreparedView view,BatchDiagnostics identity) {
  auto& s=*storage;
  s.control={};
  s.control.diagnostics=identity;
  for(std::size_t parent=0;parent<s.model.config.element_count;++parent) {
    if(s.candidate_status[parent]==Status::kSuccess) continue;
    s.control.status=BatchStatus::ElementFailure;
    s.control.element=static_cast<std::uint32_t>(parent);
    s.control.element_status=s.candidate_status[parent];
    return;
  }
  if(!qbat_gather_test::serial::Measure(s.model,*accepted,*trial,view,s.control.diagnostics)) {
    s.control.status=BatchStatus::NonfiniteResult;
    return;
  }
  s.control.diagnostics.valid=true;
}
} // namespace qbat_measurement_test::serial
