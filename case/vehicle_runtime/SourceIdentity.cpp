#include "SourceIdentity.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_runtime::detail {
void CheckSource(const Execution& execution,const Attachments& attachments) {
    using output::Require;
    const auto& model = execution.model();
    const auto& physical = execution.physical();
    Require(model.SharesStorage(attachments.physical()),
        "Initial runtime inputs must retain the exact same physical model backing");
    Require(physical.prepared() && physical.execution() && physical.catalog()->execution_sections() &&
        physical.domain()->SharesStorage(model.source_domain().domain()) &&
        physical.coefficients()->nodes().data() == model.coefficients().nodes().data() &&
        physical.execution()->rigid()->groups().data() == model.rigid_assembly().groups().data() &&
        physical.execution()->rigid()->members().data() == model.rigid_assembly().members().data() &&
        attachments.attachments().model().domain()->SharesStorage(*physical.domain()),
        "Initial runtime catalog, ledger, rigid or CIN backing differs");
    Require(&execution.resolution() == model.shell_source().references().resolution() &&
        &attachments.witnesses().binding().references().source().canonical().data() ==
        &model.shell_source().references().source().canonical().data() &&
        &attachments.witnesses().binding().shells() == &model.shell_source().shells() &&
        attachments.witnesses().runtime_mappable(),
        "Initial runtime witness roster or complete source resolution differs");
}
tl::fea::NodalCinWitnessSource Witnesses(const Attachments& source) noexcept {
    const auto& data = source.witnesses().data();
    return {&source.attachments().model(),data.ranges.data(),data.witnesses.data(),
            data.ranges.size(),data.witnesses.size()};
}
} // namespace crash::cases::vehicle_runtime::detail
