#include "AcceptedNativeGroup.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "case/vehicle_dynamics/native_contact/Group.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::output::physical_run::detail {
namespace app=cases::vehicle_dynamics::native_contact;
namespace {
NativeContactRole Role(app::Role value) {
    switch(value) {
      case app::Role::Self:return NativeContactRole::Self;
      case app::Role::MeshWall:return NativeContactRole::MeshWall;
    }
    throw std::invalid_argument("Unknown actual native interface role");
}
bool SameSource(const app::native::TransactionSourceInfo& a,const app::native::TransactionSourceInfo& b) {
    return a.available && b.available && a.source_id==b.source_id && a.topology_generation==b.topology_generation &&
        a.source_generation==b.source_generation && a.nodes==b.nodes && a.secondaries==b.secondaries &&
        a.primary_mains==b.primary_mains && a.expanded_mains==b.expanded_mains;
}
}
void CaptureNativeGroup(const cases::vehicle_dynamics::VehiclePhysicalDynamics& run,Profile profile,Values& values) {
    static_assert(NativeGroupCapacity==tl::fea::MaxNativeContactInterfaces);
    const auto* group=run.native_contact_group();
    Require(profile.native_group==bool(group),"Archive profile hides or invents an actual native contact group");
    if(!group)return;
    const auto stamp=run.accepted();const auto& step=run.last_accepted_step();
    Require(stamp.epoch && !run.has_prepared_step() && values.owner==stamp.owner_id &&
        values.stamp.epoch==stamp.epoch && step.native_contact.count==group->count(),
        "Native group capture requires a completed common physical interval");
    NativeGroupValues staged;staged.count=group->count();
    for(std::size_t i=0;i<staged.count;++i) {
        const auto& tx=group->transaction(i);const auto source=tx.source_info();const auto published=tx.accepted();
        const auto& observed=step.native_contact.interfaces[i];
        Require(source.nodes==stamp.node_count && SameSource(source,observed.source) && observed.enabled &&
            step.native_contact.roles[i]==group->role(i) && observed.attempt==values.stamp.attempt &&
            tl::fea::trial_identity::SameStamp(observed.force_base,step.base) &&
            published.available && published.force_phase_available && published.selectors.has_reference &&
            tl::fea::trial_identity::SameStamp(published.stamp,stamp) &&
            tl::fea::trial_identity::SameStamp(published.force_base_stamp,step.base),
            "Native group source/history publication differs from its actual physical force attempt");
        const auto& base=published.force_base_stamp;
        staged.entries[i]={Role(group->role(i)),{source.source_id,source.topology_generation,source.source_generation,
            source.nodes,source.secondaries,source.primary_mains,source.expanded_mains,published.generation,
            published.selectors.reference_generation,base.epoch,base.time,base.velocity_time}};
    }
    CheckNativeGroupValues(staged);values.native_group=staged;
}
}
