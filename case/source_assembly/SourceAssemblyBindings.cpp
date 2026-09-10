#include "SourceAssemblyStartupInternal.h"
#include <sstream>

namespace crash::cases::source_assembly {
namespace fe=tl::fea;
struct SourceAssemblyBindings::Impl {
    Impl(const source::SourceAssembly& s, const SourceAssemblyBindingOptions& o)
        : source(s), instance(o.source_instance_id), policy(o.material_rate_policy), spotweld(o.spotweld) {}
    source::SourceAssembly source;
    std::uint64_t instance;
    source::MaterialRatePolicy policy;
    std::optional<source::SpotweldDeclaration> spotweld;
    fe::ShellBatchBinding shells;
    fe::ShellBatchPlasticityBinding materials;
    fe::NodalRigidGroupModel rigid;
    fe::type25::Model connectors;
    fe::NodalMassBinding mass;
    std::vector<PartNativeMassLedger> parts;
};
namespace {
[[noreturn]] void ShellFailure(const fe::ShellBindingReport& report, const source::SourceAssemblyShellInput& input) {
    const char* family=report.family==fe::ShellBindingFamily::Qeph?"QEPH":
        report.family==fe::ShellBindingFamily::T3?"T3":"none";
    const auto& indices=report.family==fe::ShellBindingFamily::Qeph?input.qeph_source_parents():input.t3_source_parents();
    std::uint64_t element=0, part=0;
    if(report.family!=fe::ShellBindingFamily::None && report.parent_index<indices.size()) {
        const auto& parent=input.source().data().parents.at(indices[report.parent_index]);
        element=parent.source_id; part=parent.part_id;
    }
    const unsigned native=report.family==fe::ShellBindingFamily::Qeph?unsigned(report.qeph_status):unsigned(report.t3_status);
    std::ostringstream message;
    message<<"Assembly shell startup: family="<<family<<" EID="<<element<<" PID="<<part
           <<" family_index="<<report.parent_index<<" binding_status="<<unsigned(report.status)
           <<" native_status="<<native<<": "<<report.message;
    throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Shells,unsigned(report.status),message.str(),element,part);
}
}
SourceAssemblyBindings SourceAssemblyBindings::Prepare(const source::SourceAssembly& source,
                                                       const SourceAssemblyBindingOptions& options) {
    if(!options.source_instance_id)
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Input,0,"Assembly startup requires an explicit nonzero source instance ID");
    const auto& data=source.data();
    if(data.boundary.policy!="released_external_connections")
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Input,0,"Assembly startup requires the explicit released external boundary");
    const bool has_internal_welds=!data.internal_spotwelds.empty();
    if(has_internal_welds!=options.spotweld.has_value())
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Input,0,
            "Internal source spotwelds require an explicit matching connector contribution policy at startup");
    auto next=std::make_shared<Impl>(source,options);
    const source::SourceAssemblyShellInput shell_input(source);
    const auto shell_report=next->shells.Initialize(shell_input.input(),options.shell_limits);
    if(shell_report.status!=fe::ShellBindingStatus::Success) ShellFailure(shell_report,shell_input);
    const source::SourceAssemblyMaterialInput material_input(source,options.material_rate_policy);
    const auto material_report=data.schema==source::SectionInventorySchema ?
        next->materials.InitializeSections(next->shells,material_input.input(),options.material_limits) :
        next->materials.Initialize(next->shells,material_input.input(),options.material_limits);
    if(material_report.status!=fe::ShellPlasticityBindingStatus::Success) {
        std::ostringstream message;
        message<<"Assembly material startup: entry="<<material_report.entry<<" status="<<unsigned(material_report.status)
               <<": "<<material_report.message;
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Materials,unsigned(material_report.status),message.str());
    }
    next->parts=startup::PartLedger(data,next->shells);
    startup::Connectors(source,next->shells,options,next->connectors,next->mass);
    startup::RigidGroups(data,next->shells,options,next->rigid);
    return SourceAssemblyBindings(std::move(next));
}
const source::SourceAssembly& SourceAssemblyBindings::source() const noexcept { return impl_->source; }
std::uint64_t SourceAssemblyBindings::source_instance_id() const noexcept { return impl_->instance; }
source::MaterialRatePolicy SourceAssemblyBindings::material_rate_policy() const noexcept { return impl_->policy; }
const fe::ShellBatchBinding& SourceAssemblyBindings::shells() const noexcept { return impl_->shells; }
const fe::ShellBatchPlasticityBinding& SourceAssemblyBindings::materials() const noexcept { return impl_->materials; }
const std::vector<PartNativeMassLedger>& SourceAssemblyBindings::part_mass_ledger() const noexcept { return impl_->parts; }
const fe::NodalRigidGroupModel* SourceAssemblyBindings::rigid_groups() const noexcept {
    return impl_->rigid.prepared()?&impl_->rigid:nullptr;
}
const fe::type25::Model* SourceAssemblyBindings::connectors() const noexcept {
    return impl_->connectors.prepared()?&impl_->connectors:nullptr;
}
const fe::NodalMassBinding* SourceAssemblyBindings::combined_mass() const noexcept {
    return impl_->mass.prepared()?&impl_->mass:nullptr;
}
const source::SpotweldDeclaration* SourceAssemblyBindings::spotweld_declaration() const noexcept {
    return impl_->spotweld?&*impl_->spotweld:nullptr;
}
fe::NodalMassPartitions SourceAssemblyBindings::coefficients(std::size_t node) const noexcept {
    if(node>=impl_->shells.node_count())return {};
    if(impl_->mass.prepared())return impl_->mass.nodes()[node].coefficients;
    fe::NodalMassPartitions value;value.shell=impl_->shells.nodes()[node].native;
    value.mass=value.shell.mass;value.isotropic_inertia=value.shell.isotropic_inertia;
    return value;
}
} // namespace crash::cases::source_assembly
