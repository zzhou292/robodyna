#include "JointRuntime.h"
#include "ParticipantConfigs.h"
#include "ParticipantControls.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime::detail {
namespace {
tl::fea::type45::BatchConfig JointConfig(const Config& config,const tl::fea::NodalStamp& stamp,
    const tl::fea::ShellBatchStartup& startup,const tl::fea::NodalCinWitnessSource& witness) {
    tl::fea::type45::BatchConfig out;
    SetParticipantIdentity(out,config,stamp,startup);SetCinCounts(out,witness);
    out.profile=tl::fea::type45::BatchProfile::PhysicalAggregateV1;out.limits=config.limits.joints;
    return out;
}
void JointForecast(const tl::fea::type45::BatchConfig& config,const tl::fea::type45::Model& model,
    std::size_t additional_source_bytes,Forecast& output) {
    RequireSuccess(tl::fea::type45::Batch::Forecast(config,model,output.joints));
    output::Require(output.joints.retained_model_backing_bytes<=model.owned_payload_bytes() &&
        output.joints.incremental_host_bytes<=output.joints.startup_host_bytes &&
        output.joints.retained_model_backing_bytes==output.joints.startup_host_bytes-output.joints.incremental_host_bytes,
        "Native joint startup source partition is inconsistent");
    output.has_type45=true;output.joint_source_bytes=additional_source_bytes;
}
}
void CheckJointSource(const Execution& execution,const JointModel& joints) {
    const auto& physical=execution.model();
    const auto& model=joints.model();
    output::Require(physical.SharesStorage(joints.physical()) && model.prepared() &&
        model.domain()->SharesStorage(physical.source_domain().domain()) &&
        model.rigid_binding()->groups().data()==physical.rigid_assembly().groups().data() &&
        model.rigid_binding()->members().data()==physical.rigid_assembly().members().data() &&
        model.joints().size()==joints.source_rows().size() &&
        model.joints().size()==joints.source().data().required,
        "Runtime joints must retain the exact physical rigid/domain source and all required rows");
}
tl::fea::type45::BatchConfig ConfigureJoints(const Config& config,const Attachments& attachments,
    const tl::fea::NodalStamp& stamp) {
    return JointConfig(config,stamp,InitialTranslation(),Witnesses(attachments));
}
tl::fea::type45::BatchConfig ConfigureJoints(const Config& config,const Source& source,
    const tl::fea::NodalStamp& stamp) {
    return JointConfig(config,stamp,source.startup(),source.witness_source());
}
void ForecastJoints(const Config& config,const Execution& execution,const Attachments& attachments,
    const JointModel& joints,Forecast& output) {
    CheckJointSource(execution,joints);
    JointForecast(ConfigureJoints(config,attachments,DescriptiveStamp(config,execution)),
        joints.model(),joints.additional_owned_payload_bytes(),output);
}
void ForecastJoints(const Config& config,const Source& source,Forecast& output) {
    output::Require(source.joints()!=nullptr,"Joint forecast requires the actual source model");
    JointForecast(ConfigureJoints(config,source,DescriptiveStamp(config,source)),
        *source.joints(),source.additional_joint_source_bytes(),output);
}
}
