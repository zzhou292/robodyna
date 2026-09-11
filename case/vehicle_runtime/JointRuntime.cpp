#include "JointRuntime.h"
#include "ParticipantConfigs.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime::detail {
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
    tl::fea::type45::BatchConfig out;
    out.owner=stamp;
    out.configuration_id=config.configuration_id;
    out.qualification_id=config.qualification_id;
    out.startup=InitialTranslation();
    out.profile=tl::fea::type45::BatchProfile::PhysicalAggregateV1;
    out.cin_attachment_count=attachments.witnesses().data().ranges.size();
    out.cin_witness_count=attachments.witnesses().data().witnesses.size();
    out.limits=config.limits.joints;
    return out;
}
void ForecastJoints(const Config& config,const Execution& execution,const Attachments& attachments,
    const JointModel& joints,Forecast& output) {
    CheckJointSource(execution,joints);
    const auto configured=ConfigureJoints(config,attachments,DescriptiveStamp(config,execution));
    RequireSuccess(tl::fea::type45::Batch::Forecast(configured,joints.model(),output.joints));
    output::Require(output.joints.retained_model_backing_bytes<=joints.model().owned_payload_bytes() &&
        output.joints.incremental_host_bytes<=output.joints.startup_host_bytes &&
        output.joints.retained_model_backing_bytes==
            output.joints.startup_host_bytes-output.joints.incremental_host_bytes,
        "Native joint startup source partition is inconsistent");
    output.has_type45=true;
    output.joint_source_bytes=joints.additional_owned_payload_bytes();
}
} // namespace crash::cases::vehicle_runtime::detail
