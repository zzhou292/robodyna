#include "Internal.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"

namespace crash::cases::vehicle_wall::native::physical_detail {
void Check(const WallSource& wall, const vehicle_startup::VehicleShellReferences& refs,
           EnvelopePhysicalLimits limits) {
    Require(limits.host_bytes && limits.host_bytes <= EnvelopePhysicalLimits{}.host_bytes,
        "Invalid combined physical-source host cap");
    const auto& origin = wall.vehicle_origin();
    Require(modelio::physical_scope::HasVehicleSupports(origin.source().solid_source().data().policy) &&
        wall.declaration().profile == Profile::EnvelopeFixedElasticV1 &&
        wall.vehicle_prefix().nodes == origin.domain().node_count() &&
        wall.domain().node_count() == origin.domain().node_count()+4,
        "Combined physical source requires complete supported vehicle sources and the declared wall suffix");
    Require(&origin.source().tied_source().canonical().data() == &refs.source().canonical().data(),
        "Wall and vehicle shell references do not share immutable canonical authority");
    Require(refs.qeph_metric().profile() == vehicle_startup::QephMetricProfile::AuthenticatedSourceLength &&
        refs.qeph_metric().working_length_m() == wall.geometry().native_working_length_m,
        "Combined native source requires its authenticated working-length reference profile");
    Require(refs.counts().parents == 349645 && refs.counts().qeph_succeeded == 324094 &&
        refs.counts().t3_succeeded == 21301 && refs.counts().qbat_succeeded == 4250 &&
        refs.counts().succeeded == refs.counts().parents && !refs.counts().unresolved && !refs.counts().rejected,
        "Complete original vehicle shell family counts changed");
    Require(refs.counts().parents < limits.shells.native.max_parents &&
        refs.source().counts().nodes <= limits.shells.native.max_nodes &&
        limits.shells.native.max_nodes-refs.source().counts().nodes >= 4,
        "Combined shell input exceeds native parent/node capacities");
    model::detail::ValidateComponentLimits(limits.components, true);
}
fe::ShellQephBindingInput EnvironmentInput(const fe::qeph::ReferenceInput& reference,
    std::uint64_t source_parent, std::size_t original_shell_nodes) {
    Require(source_parent && original_shell_nodes <= fe::ShellHostBindingLimits::Vehicle().max_nodes-4,
        "Invalid environment shell packing identity or node extent");
    fe::ShellQephBindingInput parent;
    parent.source_parent_id = source_parent;
    parent.reference = reference;
    for (unsigned k=0; k<4; ++k) parent.nodes[k] = original_shell_nodes+k;
    return parent;
}
shell::Inputs Pack(const WallSource& wall, const vehicle_startup::VehicleShellReferences& refs) {
    auto inputs = shell::Pack(refs, 1);
    inputs.qeph.push_back(EnvironmentInput(wall.geometry().reference_input,
        wall.ids().shell, refs.source().counts().nodes));
    Require(inputs.qeph.size() == refs.counts().qeph_succeeded+1 &&
        inputs.t3.size() == refs.counts().t3_succeeded && inputs.qbat.size() == refs.counts().qbat_succeeded,
        "Combined packing changed an original shell family extent");
    Require(inputs.qeph.capacity() <= 2*inputs.qeph.size() &&
        inputs.t3.capacity() <= 2*inputs.t3.size() && inputs.qbat.capacity() <= 2*inputs.qbat.size(),
        "Combined shell packing capacity exceeds reservation");
    return inputs;
}
}
