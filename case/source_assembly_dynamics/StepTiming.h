#pragma once
#include "benchmarks/stage_timing/StageTimer.h"

namespace crash::cases::source_assembly_dynamics {
// Diagnostic call inventory, not a physical epoch or another integrator clock.
enum class StepStage : std::size_t {
    Step,BeginTrial,AssembleQeph,AssembleT3,AssembleWall,CopyAppliedLoads,
    SealAssembly,AdvanceOwner,ReadPreparedNodes,ReadPreparedGroups,
    EvaluateQeph,EvaluateT3,ReadQephResults,ReadT3Results,ReadQephSections,ReadT3Sections,
    EvaluateWall,ReadWallResults,PreparePublication,CheckShells,CheckContact,CheckMotion,
    CommitPublication,DiscardTrial,Count
};
inline constexpr std::size_t StepStageCount=static_cast<std::size_t>(StepStage::Count);
inline constexpr const char* StepStageNames[]{
    "step_inclusive","begin_trial","assemble_qeph","assemble_t3","assemble_wall","copy_applied_loads",
    "seal_assembly","advance_owner","read_prepared_nodes","read_prepared_groups",
    "evaluate_qeph","evaluate_t3","read_qeph_results","read_t3_results","read_qeph_sections","read_t3_sections",
    "evaluate_wall","read_wall_results","prepare_publication","check_shells","check_contact","check_motion",
    "commit_publication","discard_trial"
};
static_assert(sizeof(StepStageNames)/sizeof(*StepStageNames)==StepStageCount);
struct StepTimingOptions { bool enabled=false; };
using StepTimingSnapshot=benchmarks::StageTimingSnapshot<StepStageCount>;
class StepTimer : public benchmarks::StageTimer<StepStageCount> {
  public:
    explicit StepTimer(StepTimingOptions options={}) noexcept:StageTimer(options.enabled) {}
    template<StepStage S,class F> auto Measure(F&& call) -> decltype(call()) {
        return StageTimer::Measure<static_cast<std::size_t>(S)>(std::forward<F>(call));
    }
};
} // namespace crash::cases::source_assembly_dynamics
