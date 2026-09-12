#include "BeamRuntime.h"
#include "ParticipantConfigs.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime::detail {
void CheckBeamSource(const Execution& execution) {
    const auto& physical=execution.model();
    const auto* model=physical.structural_beams();
    const auto* coefficient=physical.coefficients().beam18();
    const bool supports=physical.source_domain().policy()==
        modelio::physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5;
    output::Require(supports==bool(model) && supports==bool(coefficient),
        "Runtime structural beam source and physical profile differ");
    if(!model) return;
    output::Require(model->prepared() && coefficient->model()->SharesStorage(*model) &&
        model->domain()->SharesStorage(physical.source_domain().domain()) &&
        physical.coefficients().order()==tl::fea::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_Beam18_V5,
        "Runtime structural beam model must share the actual V5 coefficient authority");
}
tl::fea::beam18::BatchConfig ConfigureStructuralBeams(const Config& config,const Attachments& attachments,
    const tl::fea::NodalStamp& stamp) {
    tl::fea::beam18::BatchConfig out;
    out.owner=stamp; out.configuration_id=config.configuration_id; out.qualification_id=config.qualification_id;
    out.startup=InitialTranslation();
    out.profile=tl::fea::beam18::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
    out.cin_attachment_count=attachments.witnesses().data().ranges.size();
    out.cin_witness_count=attachments.witnesses().data().witnesses.size();
    out.limits=config.limits.structural_beams;
    return out;
}
void ForecastStructuralBeams(const Config& config,const Execution& execution,const Attachments& attachments,Forecast& out) {
    CheckBeamSource(execution);
    const auto* model=execution.model().structural_beams();
    if(!model) return;
    RequireSuccess(tl::fea::beam18::Batch::Forecast(
        ConfigureStructuralBeams(config,attachments,DescriptiveStamp(config,execution)),*model,out.structural_beams));
    const auto retained=model->owned_payload_bytes();
    output::Require(retained>=sizeof(*model),"Structural beam Model retained size is invalid");
    const auto shared=retained-sizeof(*model);
    output::Require(out.structural_beams.startup_host_bytes>=shared,
        "Structural beam complete startup source partition is inconsistent");
    // SourceBytes already retains this exact Model through ledger.beam18().
    // Conservatively reserve all remaining upload/proof/readback workspace.
    out.has_beam18=true;
    out.structural_beam_incremental_host_bytes=out.structural_beams.startup_host_bytes-shared;
}
} // namespace crash::cases::vehicle_runtime::detail
