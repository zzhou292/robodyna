#pragma once
#include "SourceAssemblyBinaryFrames.h"

namespace crash::output::assembly::binary {
// Versioned component mapping, deliberately distinct from complete canonical
// full-shell mappings. Source bytes authenticate geometry/declarations; these
// ordered canonical UInt64 arrays bind native/display/source field association.
inline constexpr const char* MappingDomain="robo_dyna.source_assembly_binary_mapping.v1";
inline constexpr std::uint32_t QephFamily=1,T3Family=2;
std::string ComponentMappingDigest(const SourceAssemblySurface&);
records::Context ComponentContext(const cases::source_assembly::SourceAssemblyBindings&,
    const SourceAssemblySurface&,std::uint64_t configuration,std::uint64_t qualification,
    double fixed_dt,records::RecordLimits={});
namespace detail {
// Formatting-only seam, not acceptance proof. Context must already be bound to
// this immutable component mapping; output storage is exact and disjoint from
// every inspected view. All validation completes before any output change.
void StageFrame(const records::Context&,const wall_fields::FrameView&,records::FrameRecord&);
}
} // namespace crash::output::assembly::binary
