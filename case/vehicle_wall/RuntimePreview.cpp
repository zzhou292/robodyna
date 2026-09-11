#include "RuntimeData.h"
#include "RuntimeBudget.h"
#include "case/vehicle_runtime/ParticipantConfigs.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall {
RuntimeForecast VehicleWallStartup::Preview(const VehicleWallSetup& setup,
    vehicle_dynamics::Config config, RuntimeLimits limits,const vehicle_runtime::JointModel* joints) {
    const auto dynamics=vehicle_dynamics::VehiclePhysicalDynamics::Preflight(
        setup.execution(),setup.attachments(),config,joints);
    // Forecast reads source values only; this empty, unclaimed coordinator is a
    // prospective object, not an initialized participant/publication authority.
    tl::fea::ShellBatchPublication prospective_publication;
    const auto& execution=setup.execution();
    tlfea::contact::NodalWallMappedSource source;
    source.physical=&execution.physical();
    source.rigid=&execution.model().rigid_assembly();
    source.cin=vehicle_runtime::detail::Witnesses(setup.attachments());
    source.publication=&prospective_publication;
    source.identity={config.startup.configuration_id,config.startup.qualification_id,
                     vehicle_runtime::detail::InitialTranslation()};
    const auto stamp=vehicle_runtime::detail::DescriptiveStamp(config.startup,execution);
    const auto contact_config=ContactConfig(setup.settings(),stamp,config.startup.configuration_id,
        config.startup.qualification_id,setup.placement());
    tl::fea::ShellMappedFootprint contact;
    const auto report=tlfea::contact::NodalWallMappedContact::Forecast(contact_config,*setup.geometry().weights(),
        source,contact,limits.contact);
    output::Require(report.status==tlfea::contact::NodalWallDeviceStatus::Ok,report.message);
    return detail::ComposeForecast(dynamics,setup.forecast(),contact,
        sizeof(Data)+sizeof(VehicleWallStartup)+256,limits);
}
} // namespace crash::cases::vehicle_wall
