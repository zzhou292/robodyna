#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_dynamics/Reports.h"
namespace crash::cases::vehicle_dynamics {
void VehiclePhysicalDynamics::InstallNativeContact(std::unique_ptr<native_contact::Group> group) {
    output::Require(bool(storage_) && bool(group),"Native contact installation requires a live case and actual group");
    auto& s=*storage_;
    output::Require(!s.pending && !s.native_contact && !s.wall && !s.self_contact &&
        s.stamp.epoch==0 && tl::fea::trial_identity::SameStamp(s.stamp,s.state().owner.accepted()),
        "Native contact requires an unchanged initial owner and an exclusive contact profile");
    const auto physical=allocations().device_bytes;
    const auto cap=s.config.startup.limits.device_bytes;
    output::Require(physical<=cap && group->device_bytes()<=cap-physical,
        "Complete physical owner and native contact exceed the declared device cap");
    auto& state=s.state();
    group->Bind(state.owner,state.publication,state.source.physical(),
        {&state.qeph,&state.t3,&state.qbat,&state.type25,&state.type13,&state.solids,state.type45.get(),state.beam18.get()},
        {state.config.configuration_id,state.config.qualification_id,state.source.startup()});
    // Bind already authenticates the unchanged accepted owner. Ownership moves
    // only after its actual roster is configured; no fallible work follows.
    s.native_contact=std::move(group);
}
}
