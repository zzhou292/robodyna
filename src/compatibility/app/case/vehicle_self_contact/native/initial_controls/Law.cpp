#include "Values.h"
#include "lib_src/collision/radioss_type25/NativeConstants.h"
#include "modelio/vehicle_source/SourceCards.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
n::TransactionConfig ResolveSelfLaw(const modelio::self_contact::Data& source,n::UnitScale units) {
    (void)ResolveOriginalControls(source);
    const auto& fields=source.source_fields;
    output::Require(fields.static_friction&&fields.dynamic_friction&&fields.decay_coefficient,
        "Original self friction source fields are unavailable");
    const auto& card=source.sources[0].cards[1].second;
    const auto fs=modelio::vehicle::detail::SourceScalar(card,0);
    const auto fd=modelio::vehicle::detail::SourceScalar(card,1);
    const auto dc=modelio::vehicle::detail::SourceScalar(card,2);
    output::Require(fs&&fd&&dc&&std::isfinite(*fs)&&std::isfinite(*fd)&&std::isfinite(*dc)&&
        *fs>=0&&*fd>=0&&*dc>=0&&*fs==*fields.static_friction&&*fd==*fields.dynamic_friction&&*dc==*fields.decay_coefficient,
        "Original self friction cards differ from the retained declaration");
    n::TransactionConfig out;out.units=units;
    out.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
    out.physical_source=n::PhysicalSourceProfile::CompleteBoundLedger;
    out.activity=n::ContactActivityPolicy::AllActivePrefix;
    auto& s=out.lifecycle.selection;s.gap_mode=1;s.initial_penetration=5;s.local_processor=1;
    s.foreign_rows=false;s.thermal=false;s.gap_loading=false;
    auto& g=out.lifecycle.geometry;g.gap_mode=1;g.sharp=1;g.initial_penetration=5;g.damping_flag=1;
    g.adhesion=false;g.thermal=false;g.foreign_row=false;
    out.lifecycle.coefficient.stiffness_formulation=4;out.lifecycle.coefficient.mass_timestep_augmentation=0;
    out.lifecycle.minimum_coefficient=0;out.lifecycle.maximum_coefficient=n::native_constant::ep20*n::native_constant::ep10;
    out.lifecycle.neighbor_removal=2;out.lifecycle.optcd_response_precision=0;
    out.lifecycle.main_coefficient_domain=n::MainCoefficientDomain::NativeSigned;
    out.normal.stiffness_formulation=4;out.normal.damping_flag=1;out.normal.initial_penetration=5;
    out.normal.arithmetic_precision=8;out.normal.prescribed_contact_force=false;out.normal.adhesion=false;
    out.normal.damping_factor=5./100.; // FIVEEM2=ZEP05=FIVE/EP02 in pinned MYREAL8 constants.
    out.normal.engine.kdtint=0;out.normal.engine.idtmins=0;out.normal.engine.idtmins_int=0;
    out.friction.model=2;out.friction.formulation=10;out.friction.orthotropic=0;out.friction.converged=1;
    // INCONV1 is the explicit serial explicit-driver profile, not a contact-card value.
    out.friction.thermal=0;out.friction.part_coefficients=0;out.friction.alpha=1;
    // Original ConvertContacts, with authenticated blank FSF ->1. Preserve C6
    // negative zero rather than replacing the source expression with abs/zero.
    out.friction_coefficients.base=*fd*1.;
    out.friction_coefficients.c[4]=(*fs-*fd)*1.;
    out.friction_coefficients.c[5]=-*dc;
    out.assembly.parallel_assembly=0;out.assembly.pinch=0;out.assembly.thermal=0;
    out.assembly.thermal_formulation=0;out.assembly.thermal_nodal_timestep=0;
    out.assembly.engine.kdtint=0;out.assembly.engine.idtmins=0;out.assembly.engine.idtmins_int=0;
    return out;
}
}
