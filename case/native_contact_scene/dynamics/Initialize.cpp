#include "Internal.h"
namespace crash::cases::native_scene {
namespace dd=dynamics_detail;
void NativeSceneDynamics::Storage::Initialize() {
    const auto& physical=source.physical_source();const auto& binding=physical.physical();
    {
        vehicle_runtime::SourceRoles roles;
        // PhysicalSource admits only a complete QEPH/T3 ledger: every node has
        // genuine shell incidence and no rigid/CIN/other-family membership.
        roles.node.assign(binding.domain()->node_count(),vehicle_runtime::Shell);
        auto packing=vehicle_runtime::detail::PackOwner(*binding.coefficients(),physical.rigid(),roles,
            physical.startup().uniform_velocity,forecast.packing_bytes);
        packing.fixed=physical.translation_fixed_bits();packing.rotation_fixed=physical.rotation_fixed();
        for(std::size_t i=0;i<packing.mass.size();++i) {
            const auto v=tl::fea::shell_startup_detail::ProjectVelocity(physical.startup().uniform_velocity,packing.fixed[i]);
            packing.velocity[3*i]=v.x;packing.velocity[3*i+1]=v.y;packing.velocity[3*i+2]=v.z;
            if(packing.fixed[i]==7)packing.inverse_mass[i]=0;
            if(packing.rotation_fixed[i])packing.inverse_inertia[i]=0;
        }
        const auto cin=dd::Cin(physical,config,packing.mass.data(),packing.inertia.data());
        dd::RequireSuccess(owner.Initialize(dd::OwnerConfig(physical,config),packing.kinematics(),packing.inverse_mass.data(),
            packing.dofs(),physical.rigid(),&cin));
    }
    dd::RequireSuccess(qeph.InitializeMapped(dd::QuadConfig(physical,config,owner.accepted()),binding,owner,dd::Witnesses(physical)));
    dd::RequireSuccess(t3.InitializeMapped(dd::TriangleConfig(physical,config,owner.accepted()),binding,owner,dd::Witnesses(physical)));
    // Authenticate initial zero-stress caches from the actual accepted owner.
    // This proof is uncommitted: no seal, advance or contact history evaluation.
    tl::fea::NodalAssemblyView assembly;
    dd::RequireSuccess(owner.BeginTrial(&token,&assembly));trial=true;
    dd::RequireSuccess(qeph.AssembleMappedAccepted(owner,token,assembly));
    dd::RequireSuccess(t3.AssembleMappedAccepted(owner,token,assembly));
    Discard();
    dd::RequireSuccess(publication.InitializePhysical(owner,binding,physical.rigid(),dd::Witnesses(physical),Participants(),Identity(),
        dd::PublicationLimits(binding.domain()->node_count())));
    dd::RequireSuccess(contact.Initialize(source.config(),source.source(),owner,publication,binding,Participants(),Identity(),config.limits.contact));
    dd::RequireSuccess(publication.ConfigurePhysicalScratchParticipation(owner,binding,Participants(),Identity(),{{},contact.roster_entry()}));
    // Exact resident component allocations and conservative transaction caps
    // are different claims. No driver/context allocation size is inferred.
    output::Require(owner.allocations().device_bytes==forecast.owner.device_bytes&&
        qeph.allocations().device_bytes==forecast.qeph.device_bytes&&t3.allocations().device_bytes==forecast.t3.device_bytes&&
        publication.allocations().device_bytes==forecast.publication.device_bytes,
        "Actual owner/participant device allocation differs from preflight");
    const auto native=contact.allocations();
    output::Require(native.device_bytes<=forecast.transaction_device_reservation&&
        native.startup_host_bytes<=forecast.transaction_host_reservation,
        "Native transaction exceeds its admitted complete reservation");
}
}
