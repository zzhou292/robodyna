#include "State.h"
namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyWallCase::Impl::CheckQephSpin() {
    if(!spin)return Success();
    const auto& old=accepted();observation::QephSpinInput input;
    input.bindings=&bindings;input.source_node=config.observe_qeph_spin_node;
    input.configuration_id=setup.settings()->configuration_id;input.qualification_id=setup.settings()->qualification_id;
    input.base=old.diagnostics.stamp;input.prepared=prepared;input.before=old.fields.view();input.after=candidate().fields.view();
    input.parent_diagnostics=&old.diagnostics.shells.qeph;input.candidate_diagnostics=&candidate().diagnostics.shells.qeph;
    input.parents=old.parents.qeph.data();input.sections=old.parents.qsection.data();input.parent_count=quads();
    input.candidate_parents=candidate().parents.qeph.data();input.candidate_sections=candidate().parents.qsection.data();
    input.applied_force_xyz=applied_force.data();input.applied_couple_xyz=applied_couple.data();
    return Convert(observation::ObserveQephSpin(input,&(*spin)[1-accepted_slot]));
}
}
