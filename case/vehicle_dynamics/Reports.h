#pragma once
#include "NativeStageError.h"
#include "output/ArtifactIO.h"
#include <stdexcept>
#include <string>
namespace crash::cases::vehicle_dynamics::detail {
inline void Require(const tl::fea::solids::BatchReport& report, const char* stage) {
    if (report.status != tl::fea::solids::BatchStatus::Success)
        throw NativeStageError(report, stage);
}

inline void Require(const tl::fea::beam18::BatchReport& report, const char* stage) {
    if (report.status != tl::fea::beam18::BatchStatus::Success)
        throw NativeStageError(report, stage);
}

inline void Require(const tl::fea::qeph::BatchReport& report, const char* stage) {
    if (report.status != tl::fea::qeph::BatchStatus::Success)
        throw NativeStageError(report, stage);
}

template<class Report> void Require(const Report& r,const char* stage) {
    if(static_cast<int>(r.status)!=0) throw std::runtime_error(std::string(stage)+": "+r.message);
}
} // namespace crash::cases::vehicle_dynamics::detail
