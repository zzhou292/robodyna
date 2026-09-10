#pragma once
#include "case/source_assembly/SourceAssemblyWallSetup.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include <cstddef>
#include <cstdint>

namespace crash::cases::source_assembly_dynamics {
struct ConnectorStorageLimits {
    std::size_t max_connections=1024,max_device_bytes=2*1024*1024,max_host_bytes=8*1024*1024;
};
struct DeformationLimits {
    double maximum_displacement=0,maximum_rotation=0,maximum_rotation_increment=0;
    double maximum_strain=0,maximum_thickness_curvature=0;
    double minimum_area_ratio=0,maximum_area_ratio=0;
    double minimum_thickness_ratio=0,maximum_thickness_ratio=0;
    // Necessary native element diagnostic guard, not a coupled stability proof.
    double maximum_native_dt_fraction=0;
};
struct StorageLimits {
    std::size_t max_nodes=2048,max_parents=1024,max_host_bytes=32*1024*1024;
    std::size_t max_device_bytes=32*1024*1024;
    std::size_t owner_device_bytes=1024*1024,qeph_device_bytes=4*1024*1024,t3_device_bytes=4*1024*1024;
    tl::fea::ShellPublicationLimits publication{2048,128*1024,1024*1024};
    source_assembly::SourceAssemblyWallDeviceLimits contact{};
    ConnectorStorageLimits connector;
};
enum class RotationDomain : std::uint8_t { NodalQuaternion,NativeShellGeometryV1 };
struct Config {
    double fixed_dt=0;
    DeformationLimits deformation;
    StorageLimits storage;
    // Optional base-time observation from the existing native kick; no extra force evaluation.
    bool observe_force_stage=false;
    RotationDomain rotation_domain=RotationDomain::NodalQuaternion;
    // Optional complete incident QEPH force-stage probe for one ordinary source node.
    std::uint64_t observe_qeph_spin_node=0;
};
enum class Status { Ok,InvalidInput,AlreadyInitialized,NotInitialized,ResourceLimit,
    SourceMismatch,ComponentFailure,DeviceFailure,EnvelopeFailure,ObservationFailure };
struct Report {
    Status status=Status::InvalidInput;
    const char* message="Invalid source assembly dynamics request";
    std::uint64_t source_parent=0;
    std::size_t node=SIZE_MAX;
    double measured=0,limit=0;
    explicit operator bool() const noexcept { return status==Status::Ok; }
};
bool ValidConfig(const Config&) noexcept;
} // namespace crash::cases::source_assembly_dynamics
