#pragma once
#include "SourceAssemblyInitialKinetic.h"
#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include "case/PlacedCanonicalWall.h"
#include "case/wall_penalty/WallPenaltyCertification.h"
#include "lib_src/collision/NodalWallContactDevice.h"

namespace crash::cases::source_assembly {
enum class SourceAssemblyWallBoundary { Unspecified,ReleasedExternalConnections };
struct SourceAssemblyWallSettings {
    std::array<double,3> initial_velocity{1,0,0};
    double leading_gap=.02,motion_margin=.02,exposed_clearance=1e-6;
    wall_penalty::PenaltyDesign penalty{1e-5,.0015,.002,1.10};
    double parent_force_error=1e-6,parent_energy_error=1e-9,maximum_step_rate=.125;
    std::uint64_t configuration_id=0,qualification_id=0,wall_binding_id=0;
    SourceAssemblyWallBoundary boundary=SourceAssemblyWallBoundary::Unspecified;
};
struct SourceAssemblyWallLimits {
    ShellContactGeometryLimits geometry{};
    // Additional setup startup payload: charge the complete geometry budget,
    // wall manifest copies and a bounded wall/group staging reserve. The already
    // owned authenticated bindings are retained by shared immutable handle.
    std::size_t max_startup_bytes=48*1024*1024,max_wall_manifest_bytes=1024*1024;
};
struct SourceAssemblyWallDeviceLimits {
    tlfea::contact::NodalWallDeviceLimits counts{1024,2048,2048};
    std::size_t max_device_bytes=tlfea::contact::MaxActiveNodalWallDeviceBytes;
    std::size_t max_host_bytes=tlfea::contact::MaxNodalWallHostBytes;
};
struct SourceAssemblyWallCertificate {
    SourceAssemblyInitialKinetic initial_kinetic;
    wall_penalty::PenaltyCertificate penalty;
    tlfea::contact::Q4IntegralInterval leading_gap;
    tlfea::contact::PlanarWallBoxCoverage coverage;
};
enum class SourceAssemblyWallStatus { Ok,InvalidInput,AlreadyInitialized,NotInitialized,ResourceLimit,
    GeometryFailure,CertificateFailure,ScopeMismatch };
struct SourceAssemblyWallReport {
    SourceAssemblyWallStatus status=SourceAssemblyWallStatus::InvalidInput;
    const char* message="Invalid assembly wall request";
    std::size_t node=SIZE_MAX,parent=SIZE_MAX;
    explicit operator bool() const noexcept { return status==SourceAssemblyWallStatus::Ok; }
};
// Immutable host source/wall preparation. No owner, state, clock, force call or
// inverse-M/J owner is introduced. Original source, internal groups and released
// frontier remain retained together; only the wall receives declared X placement.
class SourceAssemblyWallSetup {
  public:
    SourceAssemblyWallSetup();
    ~SourceAssemblyWallSetup();
    SourceAssemblyWallSetup(const SourceAssemblyWallSetup&) noexcept=default;
    SourceAssemblyWallSetup(SourceAssemblyWallSetup&& other) noexcept
        : SourceAssemblyWallSetup(static_cast<const SourceAssemblyWallSetup&>(other)) {}
    SourceAssemblyWallSetup& operator=(const SourceAssemblyWallSetup&)=delete;
    SourceAssemblyWallSetup& operator=(SourceAssemblyWallSetup&&)=delete;
    SourceAssemblyWallReport Initialize(const SourceAssemblyBindings&,const case_data::CanonicalWall&,
        const std::string& authenticated_wall_bytes,const SourceAssemblyWallSettings&,
        const SourceAssemblyWallLimits& = {});
    bool initialized() const noexcept;
    const SourceAssemblyBindings* bindings() const noexcept;
    const SourceAssemblyWallSettings* settings() const noexcept;
    const SourceAssemblyWallCertificate* certificate() const noexcept;
    const ShellCollectionContactGeometry* source_geometry() const noexcept;
    const case_data::PlacedCanonicalWall* placed_wall() const noexcept;
    std::size_t startup_payload_bytes() const noexcept;
    // Declaration check only: full fresh stamp and exact group scope, followed
    // by normal TL native coefficient and live-owner/token authentication at
    // contributor assembly. Count/source-instance equality is not live-owner
    // authentication. Actual allocation-fit admission remains owned by TL.
    SourceAssemblyWallReport MakeDeviceConfig(const tl::fea::NodalStamp&,
        tlfea::contact::NodalWallDeviceConfig*,const SourceAssemblyWallDeviceLimits& = {}) const;
  private:
    struct Data;std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::source_assembly
