#pragma once
#include "chrono/GuidedPlateExperiment.h"
#include "output/GuidedExperimentMetadata.h"

namespace crash::case_data {
inline const char* GuidedExperimentName(reference::GuidedPlateExperiment experiment) {
    const auto* spec=reference::FindGuidedPlateExperiment(experiment);
    output::Require(spec&&output::guided_experiment_metadata::Known(spec->name),"Unknown guided experiment");
    output::Require(output::guided_experiment_metadata::Qualification(spec->name)==spec->qualification_id&&
        output::guided_experiment_metadata::Penalty(spec->name)==spec->stiffness_per_area&&
        output::guided_experiment_metadata::MaximumPenetration==spec->maximum_penetration&&
        output::guided_experiment_metadata::TargetPenetration==spec->target_penetration&&
        output::guided_experiment_metadata::ForceError==spec->force_error&&
        output::guided_experiment_metadata::PotentialError==spec->energy_error,"Guided source/protocol experiment mismatch");
    return spec->name;
}
inline bool ParseGuidedExperiment(std::string_view name,reference::GuidedPlateExperiment& out) noexcept {
    using Experiment=reference::GuidedPlateExperiment;
    if(name==output::guided_experiment_metadata::Original){out=Experiment::Original;return true;}
    if(name==output::guided_experiment_metadata::PenaltyMargin){out=Experiment::PenaltyMarginV1;return true;}
    return false;
}
inline bool GuidedExperimentIdentity(reference::GuidedPlateExperiment experiment,std::uint64_t qualification) noexcept {
    const auto* spec=reference::FindGuidedPlateExperiment(experiment);return spec&&qualification==spec->qualification_id;
}
} // namespace crash::case_data
