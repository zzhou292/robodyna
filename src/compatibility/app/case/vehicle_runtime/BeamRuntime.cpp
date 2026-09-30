#include "BeamRuntime.h"
#include "ParticipantConfigs.h"
#include "ParticipantControls.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime::detail {
namespace {
tl::fea::beam18::BatchConfig BeamConfig(const Config& config,const tl::fea::NodalStamp& stamp,
    const tl::fea::ShellBatchStartup& startup,const tl::fea::NodalCinWitnessSource& witness) {
    tl::fea::beam18::BatchConfig out;
    SetParticipantIdentity(out,config,stamp,startup);SetCinCounts(out,witness);
    out.profile=tl::fea::beam18::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
    out.limits=config.limits.structural_beams;return out;
}
void BeamForecast(const tl::fea::beam18::BatchConfig& config,const tl::fea::beam18::Model& model,Forecast& out) {
    RequireSuccess(tl::fea::beam18::Batch::Forecast(config,model,out.structural_beams));
    const auto retained=model.owned_payload_bytes();
    output::Require(retained>=sizeof(model),"Structural beam Model retained size is invalid");
    const auto shared=retained-sizeof(model);
    output::Require(out.structural_beams.startup_host_bytes>=shared,
        "Structural beam complete startup source partition is inconsistent");
    // The authenticated source ledger already retains this exact Model.
    out.has_beam18=true;out.structural_beam_incremental_host_bytes=out.structural_beams.startup_host_bytes-shared;
}
}
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
    return BeamConfig(config,stamp,InitialTranslation(),Witnesses(attachments));
}
tl::fea::beam18::BatchConfig ConfigureStructuralBeams(const Config& config,const Source& source,
    const tl::fea::NodalStamp& stamp) {
    return BeamConfig(config,stamp,source.startup(),source.witness_source());
}
void ForecastStructuralBeams(const Config& config,const Execution& execution,const Attachments& attachments,Forecast& out) {
    CheckBeamSource(execution);
    if(const auto* model=execution.model().structural_beams())
        BeamForecast(ConfigureStructuralBeams(config,attachments,DescriptiveStamp(config,execution)),*model,out);
}
void ForecastStructuralBeams(const Config& config,const Source& source,Forecast& out) {
    const auto* model=source.structural_beams();
    if(!model)return;
    const auto* coefficient=source.coefficients().beam18();
    output::Require(coefficient && coefficient->model()->SharesStorage(*model) &&
        model->domain()->SharesStorage(*source.physical().domain()),"Runtime structural beam source differs");
    BeamForecast(ConfigureStructuralBeams(config,source,DescriptiveStamp(config,source)),*model,out);
}
}
