#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::BindInitialCaches() {
    tl::fea::NodalTrialToken token;
    tl::fea::NodalAssemblyView assembly;
    detail::RequireSuccess(owner.BeginTrial(&token,&assembly));
    try {
        detail::RequireSuccess(qeph.AssembleMappedAccepted(owner,token,assembly));
        detail::RequireSuccess(t3.AssembleMappedAccepted(owner,token,assembly));
        detail::RequireSuccess(qbat.AssembleMappedAccepted(owner,token,assembly));
        detail::RequireSuccess(type25.AssembleMappedAccepted(owner,token,assembly));
        detail::RequireSuccess(type13.AssembleMappedAccepted(owner,token,assembly));
    } catch (...) {
        owner.Discard();
        throw;
    }
    owner.Discard();
    qeph.DiscardTrial();
    t3.DiscardTrial();
    qbat.DiscardTrial();
    type25.DiscardTrial();
    type13.DiscardTrial();
    // No SealAssembly, kick, drift or accepted-history publication occurs.
}
void VehiclePhysicalStartup::Storage::InitializePublication() {
    const auto c = detail::ConfigureParticipants(config,source,owner.accepted());
    if(type45) {
        detail::RequireSuccess(publication.InitializePhysicalWithJoints(owner,source.physical(),
            source.rigid(),source.witness_source(),(*source.joints()),
            {&qeph,&t3,&qbat,&type25,&type13,&solids,type45.get(),beam18.get()},c.publication,config.limits.publisher));
        return;
    }
    detail::RequireSuccess(publication.InitializePhysical(owner,source.physical(),
        source.rigid(),source.witness_source(),
        {&qeph,&t3,&qbat,&type25,&type13,&solids,nullptr,beam18.get()},c.publication,config.limits.publisher));
}
} // namespace crash::cases::vehicle_runtime
