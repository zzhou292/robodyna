// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupMath.h"
#include <cstdint>
#include <memory>

namespace tl::fea {
struct NodalRigidGroupMember {
  std::uint64_t source_node_id=0;
  std::size_t global_node=0;
  tl::math::Vec3 position{}; // Exact supplied SI reference coordinates.
  double mass_kg=0,total_inertia_kg_m2=0,physical_inertia_kg_m2=0,added_inertia_kg_m2=0;
  // Direct source contribution whose physical/added split is unavailable.
  // This is attribution evidence, never a replacement for authoritative total J.
  double unpartitioned_native_inertia_kg_m2=0;
};
struct NodalRigidGroupInput {
  std::uint64_t source_group_id=0,source_node_set_id=0;
  const NodalRigidGroupMember* members=nullptr;
  std::size_t member_count=0;
};
// Required explicit declaration: all member data is SI, while the donor's
// generated-primary Mass/Jxx/Jyy/Jzz values 1e-20 are in these source units.
// Original Yaris t/mm/s uses {1000,.001}. Zero/default is deliberately invalid.
struct NodalRigidSourceUnits { double mass_to_kg=0,length_to_m=0; };
struct NodalRigidGroupLimits {
  std::size_t max_groups=64,max_members=4096,max_members_per_group=256;
  // Owned host payload including temporary identity-index storage, excluding
  // allocator metadata. No CUDA allocation or runtime capacity is changed.
  std::size_t max_host_bytes=8*1024*1024;
  static constexpr NodalRigidGroupLimits Vehicle() noexcept {return {1024,8192,256,8*1024*1024};}
};
struct NodalRigidGroupModelInput {
  std::uint64_t source_instance_id=0;
  std::size_t global_node_count=0;
  const NodalRigidGroupInput* groups=nullptr;
  std::size_t group_count=0;
  NodalRigidSourceUnits source_units{};
  NodalRigidGroupLimits limits{};
};
struct NodalRigidRegularizationLedger {
  double primary_mass_kg=0,primary_isotropic_inertia_kg_m2=0;
  tl::math::Vec3 principal_inertia_added{};
  tl::math::Matrix3 tensor_added{};
  bool principal_threshold_reached=false,principal_inertia_changed=false;
};
struct NodalRigidGroupProperties {
  std::uint64_t source_group_id=0,source_node_set_id=0;
  std::size_t member_offset=0,member_count=0;
  double structural_mass_kg=0,total_mass_kg=0;
  // Original native total J is summed independently and remains authoritative.
  double native_total_inertia_sum=0,physical_inertia_sum=0,added_inertia_sum=0;
  tl::math::Vec3 generated_primary_position{},structural_center{},center{};
  tl::math::Matrix3 raw_tensor{},effective_tensor{};
  rigid::PrincipalFrame principal{};
  tl::math::Vec3 raw_principal_inertia{};
  NodalRigidRegularizationLedger regularization{};
  double unpartitioned_native_inertia_sum=0;
};
enum class NodalRigidGroupStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,DuplicateIdentity,
  DuplicateMembership,InvalidMass,NonfiniteResult,EigenFailure
};
struct NodalRigidGroupReport {
  NodalRigidGroupStatus status=NodalRigidGroupStatus::Success;
  const char* message="";
  std::size_t group=SIZE_MAX,member=SIZE_MAX;
  explicit operator bool() const noexcept { return status==NodalRigidGroupStatus::Success; }
};

// Immutable startup model for fully supplied disjoint plain nodal-rigid groups.
// No source parser, state owner, timestep, constraint projection, dynamics or
// commit API. Inputs remain readable until Initialize returns. It copies every
// member in original order and never adds generated primary nodes to the source
// physical-node inventory. All failures leave this model unchanged.
class NodalRigidGroupModel {
 public:
  NodalRigidGroupModel();
  ~NodalRigidGroupModel();
  NodalRigidGroupModel(const NodalRigidGroupModel&)=delete;
  NodalRigidGroupModel& operator=(const NodalRigidGroupModel&)=delete;
  NodalRigidGroupReport Initialize(const NodalRigidGroupModelInput&) noexcept;
  // Explicit source coefficients from shells/solids/point masses. Individual
  // members may have M0/J0; the complete body's structural mass must be positive.
  // Native primary/tensor corrections are unchanged. Runtime requires the
  // complete physical assembly binding, not the legacy plain-owner overload.
  NodalRigidGroupReport InitializePhysical(const NodalRigidGroupModelInput&) noexcept;
  // Explicit physical assembly containing native-total-only source inertia.
  // Requires physical + added + unpartitioned evidence to match total J.
  // Legacy entry points require zero unpartitioned evidence.
  NodalRigidGroupReport InitializeNativeTotal(const NodalRigidGroupModelInput&) noexcept;
  bool physical_coefficients() const noexcept;
  bool prepared() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  NodalRigidSourceUnits source_units() const noexcept;
  std::size_t global_node_count() const noexcept;
  std::size_t group_count() const noexcept;
  std::size_t member_count() const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  const NodalRigidGroupProperties* groups() const noexcept;
  const NodalRigidGroupMember* members() const noexcept;
 private:
  NodalRigidGroupReport InitializeImpl(const NodalRigidGroupModelInput&, bool physical,
      bool allow_unpartitioned=false) noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea
