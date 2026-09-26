#pragma once
#include "Forecast.h"
#include <memory>
namespace crash::cases::vehicle_dynamics { struct ExecutionAccess; }
namespace crash::cases::vehicle_startup::joints { class VehicleJointModel; }
namespace crash::cases::vehicle_runtime {
using JointModel = vehicle_startup::joints::VehicleJointModel;
namespace detail { struct CaptureAccess; }
struct InitialInspection {
    tl::fea::NodalStamp stamp;
    tl::fea::NodalAllocationInfo allocations;
    std::size_t nodes = 0, absent_rotations = 0, cin_secondaries = 0;
    std::size_t shell_parents = 0, material_points = 0, rigid_skins = 0;
    std::size_t one_point_parents = 0, three_point_parents = 0, four_point_parents = 0;
    std::size_t type25_connections = 0, type13_connections = 0, solid_parents = 0;
    std::size_t type45_joints = 0, structural_beams = 0;
    std::size_t zero_point_parents = 0;
};
// One initial physical owner and its complete immutable source composition.
// The public surface deliberately has no interval advance or mutable owner.
// Construction and inspection preserve epoch/time zero; joints/contact and a
// full vehicle trajectory require subsequent explicit case admission.
class VehiclePhysicalStartup {
  public:
    static Forecast Preflight(const Source&,Config = {});
    static VehiclePhysicalStartup Prepare(const Source&,Config = {});
    static Forecast Preflight(const Execution&,const Attachments&,Config = {},const JointModel* = nullptr);
    static VehiclePhysicalStartup Prepare(const Execution&,const Attachments&,Config = {},const JointModel* = nullptr);
    ~VehiclePhysicalStartup();
    VehiclePhysicalStartup(VehiclePhysicalStartup&&) noexcept;
    VehiclePhysicalStartup& operator=(VehiclePhysicalStartup&&) noexcept;
    VehiclePhysicalStartup(const VehiclePhysicalStartup&) = delete;
    VehiclePhysicalStartup& operator=(const VehiclePhysicalStartup&) = delete;
    const Forecast& forecast() const noexcept;
    const Source& source() const noexcept;
    // Original-only adapters reject an explicitly combined environment source.
    const Execution& execution() const;
    const Attachments& attachments() const;
    tl::fea::NodalStamp accepted() const noexcept;
    tl::fea::NodalAllocationInfo allocations() const noexcept;
    // Complete native readbacks are visited one family at a time. The result is
    // returned only after all channels and exact immutable role identities pass.
    InitialInspection InspectInitial();
  private:
    friend struct vehicle_dynamics::ExecutionAccess;
    friend struct detail::CaptureAccess;
    struct Storage;
    explicit VehiclePhysicalStartup(std::unique_ptr<Storage>);
    std::unique_ptr<Storage> storage_;
};
} // namespace crash::cases::vehicle_runtime
