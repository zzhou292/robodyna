#pragma once

#include <cstddef>
#include <cstdint>

// Borrowed physical-node views for conventional elements. These do not change
// FEStateBuffer's legacy ANCF coefficient layout or take ownership of memory.
// Every batch indexes one shared physical node space through its connectivity.
// Position and velocity-level quantities are distinct; angular velocity is not
// another position coefficient. All vectors are in world coordinates and SI.
namespace tl::fea {

struct HostNodalKinematicsView {
  const double* position_xyz = nullptr;          // interleaved xyz, metres
  const double* velocity_xyz = nullptr;          // interleaved xyz, metres/s
  const double* angular_velocity_xyz = nullptr;  // interleaved xyz, radians/s
  std::size_t node_count = 0;
};

struct DeviceNodalKinematicsView {
  const double* position_xyz = nullptr;
  const double* velocity_xyz = nullptr;
  const double* angular_velocity_xyz = nullptr;
  std::size_t node_count = 0;
  std::uint64_t base_epoch = 0;
};

struct DeviceNodalForceView {
  double* force_x = nullptr;  // Each component has node_count entries, newtons.
  double* force_y = nullptr;
  double* force_z = nullptr;
  double* couple_x = nullptr; // Newton metres; conjugate to angular velocity.
  double* couple_y = nullptr;
  double* couple_z = nullptr;
  std::size_t node_count = 0;
  std::uint64_t base_epoch = 0;
};

// Device pointers remain borrowed until their stream has completed. Epochs
// identify provenance, not ownership or permission to commit. The coordinator
// validates owner/epoch and publishes a complete step once. It clears shared
// force/couple arrays once before all additive element/contact contributions.
// Time staggering belongs to the operation request, not the geometry view.
}  // namespace tl::fea
