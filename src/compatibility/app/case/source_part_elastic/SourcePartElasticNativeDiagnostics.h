#pragma once
#include "SourcePartElasticTestSupport.h"

namespace crash::cases::source_part_elastic::test {
// Failure-only, bounded diagnostic. It never changes inputs, oracle thresholds,
// accepted history or the native/CUDA comparison being investigated.
void PrintT3Failure(std::uint64_t source_parent,const tn::Reference&,const tn::HistoryValues& native_base,
    const tn::HistoryValues* previous_cuda,const tn::HistoryStamp* previous_cuda_stamp,
    const tn::PrescribedInterval&,const tn::ForceTrial& native_trial,const t::ForceTrial& cuda_trial);
} // namespace crash::cases::source_part_elastic::test
