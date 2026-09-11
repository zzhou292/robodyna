#pragma once
#include "benchmarks/stage_timing/StageTimer.h"
namespace crash::cases::vehicle_dynamics {
enum class StepStage : std::size_t {
    PrepareStep,AcceptedWitness,BeginTrial,AssembleQeph,AssembleT3,AssembleQbat,
    AssembleType25,AssembleType13,AssembleSolids,AssembleType45,AssembleWall,
    UploadWitness,SealAssembly,AdvanceCin,BorrowPrepared,EvaluateQeph,EvaluateT3,
    EvaluateQbat,EvaluateType25,EvaluateType13,EvaluateSolids,EvaluateType45,
    PreparePublication,EvaluateWall,CaptureFields,ObserveMotion,Commit,Discard,Count
};
inline constexpr std::size_t StepStageCount=static_cast<std::size_t>(StepStage::Count);
inline constexpr const char* StepStageNames[]{
    "prepare_step_inclusive","accepted_witness","begin_trial","assemble_qeph","assemble_t3","assemble_qbat",
    "assemble_type25","assemble_type13","assemble_solids","assemble_type45","assemble_wall",
    "upload_witness","seal_assembly","advance_cin","borrow_prepared","evaluate_qeph","evaluate_t3",
    "evaluate_qbat","evaluate_type25","evaluate_type13","evaluate_solids","evaluate_type45",
    "prepare_publication","evaluate_wall","capture_fields","observe_motion","commit","discard"
};
static_assert(sizeof(StepStageNames)/sizeof(*StepStageNames)==StepStageCount);
struct StepTimingOptions {bool enabled=false;};
using StepTimingSnapshot=benchmarks::StageTimingSnapshot<StepStageCount>;
class StepTimer : public benchmarks::StageTimer<StepStageCount> {
  public:
    explicit StepTimer(StepTimingOptions options={}) noexcept : StageTimer(options.enabled) {}
    template<StepStage S,class F> auto Measure(F&& call) -> decltype(call()) {
        return StageTimer::Measure<static_cast<std::size_t>(S)>(std::forward<F>(call));
    }
};
} // namespace crash::cases::vehicle_dynamics
