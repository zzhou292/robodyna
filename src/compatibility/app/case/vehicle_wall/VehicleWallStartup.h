#pragma once
#include "VehicleWallSetup.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "lib_src/collision/NodalWallMappedContact.h"
namespace crash::cases::vehicle_self_contact {
class LoadedWallSelfContact;
}
namespace crash::cases::vehicle_wall {
class LoadedWall;
struct RuntimeLimits {
    std::size_t host_bytes=std::size_t{20}*1000*1000*1000;
    std::size_t device_bytes=std::size_t{8}<<30;
    tlfea::contact::NodalWallMappedLimits contact=tlfea::contact::NodalWallMappedLimits::Vehicle();
    tl::fea::ShellPhysicalScratchParticipationLimits participation;
};
struct RuntimeForecast {
    tl::fea::ShellMappedFootprint contact;
    tl::fea::ShellPhysicalScratchParticipationForecast participation;
    std::size_t retained_host_upper_bound=0,peak_host_upper_bound=0,device_bytes=0;
};
// Initial contact source/activity authentication on the actual existing owner.
// This does not enable contact in the free-flight PrepareStep path. A later
// explicit wall dynamics hook owns ordered assembly/candidate/discard calls.
// Serialize with dynamics; its existing owner must outlive this scratch handle.
class VehicleWallStartup {
  public:
    // Value-only preview before creating the actual owner. No owner identity or
    // contact admission is granted by this descriptive allocation forecast.
    static RuntimeForecast Preview(const VehicleWallSetup&,vehicle_dynamics::Config={},RuntimeLimits={},
        const vehicle_runtime::JointModel* = nullptr);
    static RuntimeForecast Preflight(const VehicleWallSetup&,vehicle_dynamics::VehiclePhysicalDynamics&,RuntimeLimits={});
    static VehicleWallStartup Prepare(const VehicleWallSetup&,vehicle_dynamics::VehiclePhysicalDynamics&,RuntimeLimits={});
    ~VehicleWallStartup();
    VehicleWallStartup(VehicleWallStartup&&) noexcept;
    VehicleWallStartup& operator=(VehicleWallStartup&&) noexcept;
    VehicleWallStartup(const VehicleWallStartup&)=delete;
    VehicleWallStartup& operator=(const VehicleWallStartup&)=delete;
    const RuntimeForecast& forecast() const noexcept;
    const VehicleWallSetup& setup() const noexcept;
    const tl::fea::NodalStamp& initial_stamp() const noexcept;
    tl::fea::NodalAllocationInfo contact_allocations() const noexcept;
  private:
    friend class LoadedWall;
    friend class crash::cases::vehicle_self_contact::LoadedWallSelfContact;
    struct Data;
    static VehicleWallStartup PrepareUnconfigured(
        const VehicleWallSetup&,vehicle_dynamics::VehiclePhysicalDynamics&,
        RuntimeLimits);
    tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;
    explicit VehicleWallStartup(std::unique_ptr<Data>);
    std::unique_ptr<Data> data_;
};
} // namespace crash::cases::vehicle_wall
