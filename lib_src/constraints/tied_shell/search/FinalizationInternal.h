#pragma once
#include "TiedSearchFinalization.h"
namespace tl::constraints::tied_shell::finalization_detail {
FinalizationReport Preflight(const FinalizationInput&, const FinalizedSearch&, FinalizationLimits, std::size_t&) noexcept;
FinalizationReport Check(const FinalizationInput&);
void Build(const FinalizationInput&, FinalizationMaps&);
std::size_t Owned(const FinalizationMaps&) noexcept;
inline FinalizationReport Fail(FinalizationStatus status, const char* message, std::size_t row = SIZE_MAX) {
  return {status,row,message};
}
}
