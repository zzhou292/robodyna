#pragma once
#include <cstddef>
namespace tl::fea {
inline constexpr std::size_t MaxActiveRigidGroups=1024;
inline constexpr std::size_t MaxRigidMembersPerGroup=256;
inline constexpr std::size_t MaxRigidAssemblyMembersPerGroup=1024;
// Preserve the previous effective 64*256 limit for custom legacy models.
inline constexpr std::size_t MaxActiveRigidMembers=64*MaxRigidMembersPerGroup;
inline constexpr std::size_t MaxRigidOwnerHostBytes=8*1024*1024;
enum class NodalRigidOwnerProfile { PlainGroups, PartAssembly };
struct NodalRigidOwnerLimits {
  std::size_t max_groups=64,max_members=MaxActiveRigidMembers;
  // RigidStorage object and retained vector payloads only. The common nodal
  // staging/slabs stay in StateLayout; the caller's immutable model is separate.
  // No transient rigid workspace is allocated during startup or stepping.
  std::size_t max_host_bytes=MaxRigidOwnerHostBytes;
  NodalRigidOwnerProfile profile=NodalRigidOwnerProfile::PlainGroups;
  static constexpr NodalRigidOwnerLimits Vehicle() noexcept {return {1024,8192,MaxRigidOwnerHostBytes};}
  static constexpr NodalRigidOwnerLimits VehicleAssembly() noexcept {
    return {MaxActiveRigidGroups,MaxActiveRigidMembers,MaxRigidOwnerHostBytes,NodalRigidOwnerProfile::PartAssembly};
  }
};
} // namespace tl::fea
