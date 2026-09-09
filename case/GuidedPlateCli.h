#pragma once
#include "GuidedPlateRun.h"
#include "GuidedPlateWallStudyIO.h"

namespace crash::case_data {
enum class GuidedPlateCommandKind { Run,CompareRefinement,CompareWall };
struct GuidedPlateCommand {
    GuidedPlateCommandKind kind=GuidedPlateCommandKind::Run;
    GuidedPlateRunOptions run;
    std::filesystem::path coarse_study,fine_study,refinement_report;
    GuidedPlateWallStudyPaths wall_comparison;
};
// Pure bounded parsing. Existing run/compare positional forms remain accepted.
GuidedPlateCommand ParseGuidedPlateCommand(int argc,const char* const* argv);
// Exit0 success/complete horizon,2 a valid failed comparison, exceptions for
// malformed/incomplete work (main reports exit1). No automatic next run.
int ExecuteGuidedPlateCommand(const GuidedPlateCommand&);
} // namespace crash::case_data
