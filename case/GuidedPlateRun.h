#pragma once
#include "GuidedPlateCase.h"
#include <filesystem>
#include <optional>

namespace crash::case_data {
struct GuidedPlateRunOptions {
    std::filesystem::path wall,study;
    std::optional<std::filesystem::path> bundle,wall_provenance;
    WallTessellationKind wall_kind=WallTessellationKind::Original;
    // Existing typed case admission owns the execution choice. Scalar remains
    // the default; a candidate backend requires an explicit CLI option.
    GuidedPlateConfig config;
};
// Semantic/path preflight only, no device/context/state initialization. Rejects
// derived canonical bundles, missing provenance, existing outputs and aliases.
void CheckGuidedPlateRunOptions(const GuidedPlateRunOptions&);
// The existing single accepted Step/observer loop. No new state or clock.
// Completed horizon returns0 even if rebound was not observed; numerical
// comparison is a separate gate. Failure throws, retaining failed artifacts.
int RunGuidedPlate(const GuidedPlateRunOptions&);
} // namespace crash::case_data
