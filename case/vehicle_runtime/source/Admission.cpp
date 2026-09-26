#include "Storage.h"
#include "../JointRuntime.h"
#include "../Reports.h"
namespace crash::cases::vehicle_runtime {
Source Source::Original(const Execution& execution, const Attachments& attachments,
    const vehicle_startup::joints::VehicleJointModel* joints) {
    detail::CheckSource(execution,attachments);
    if(joints) detail::CheckJointSource(execution,*joints);
    Data::Original input{execution,attachments,{}};
    if(joints) input.joints.emplace(*joints);
    return Source(std::make_shared<Data>(std::move(input)));
}
Source Source::WithEnvironment(const vehicle_wall::native::EnvelopeOwnerSource& source) {
    using output::Require;
    const auto& execution=source.execution_source();
    const auto& model=execution.mechanical();
    const auto& physical=source.physical();
    const auto* bound=physical.execution();
    Require(physical.prepared() && bound && physical.catalog() && physical.catalog()->execution_sections() &&
        physical.domain()->SharesStorage(model.domain()) && physical.coefficients()->Matches(model.coefficients()) &&
        bound->rigid() && bound->rigid()->groups().data()==model.rigid_assembly().groups().data() &&
        bound->rigid()->members().data()==model.rigid_assembly().members().data() &&
        source.attachments().model().domain()->SharesStorage(model.domain()) &&
        source.witnesses().runtime_mappable(),
        "Combined runtime requires the exact qualified physical/CIN/witness graph");
    const auto& joint=source.joints();
    Require(joint.prepared() && joint.domain()->SharesStorage(model.domain()) && joint.rigid_binding() &&
        joint.rigid_binding()->groups().data()==model.rigid_assembly().groups().data() &&
        joint.rigid_binding()->members().data()==model.rigid_assembly().members().data() &&
        joint.joints().size()==source.joint_source_rows().size() &&
        joint.joints().size()==source.joint_source().data().required,
        "Combined runtime joint source is incomplete or foreign");
    const auto* beam=model.coefficients().beam18();
    Require(beam && beam->model()->SharesStorage(model.structural_beams()) &&
        model.structural_beams().domain()->SharesStorage(model.domain()) &&
        source.roles().node.size()==model.domain().node_count() &&
        source.startup().kind==tl::fea::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation &&
        tl::fea::shell_startup_detail::ValidStartup(source.startup(),true,true),
        "Combined runtime beam/domain/constraint declaration differs");
    return Source(std::make_shared<Data>(source));
}
}
