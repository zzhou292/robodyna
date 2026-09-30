// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../math/Fixed3.h"
#include "Type25Capacity.h"
#include <cstddef>
#include <cstdint>

namespace tl::fea::type25 {
using tl::math::Vec3;
using tl::math::Matrix3;

enum class Status { Success,InvalidInput,NonfiniteResult,DegenerateGeometry,ResourceLimit,DuplicateIdentity };
enum class Channel : unsigned { Axial=0,Shear=1,Torsion=2,Bending=3 };

// Required source working units for the donor's dimensional regularizers.
// All public geometry, properties, loads, histories and durations below are SI.
struct SourceUnits { double mass_to_kg=0,length_to_m=0,time_to_s=0; };

// Linear TYPE25 /SPR_AXI subset: Ileng=0, no sensor, curve, initial preload,
// rate-dependent failure or mass scaling. Values must be fully resolved by the
// source adapter; this model never selects converter defaults or material data.
// Translational channels use N/m and N*s/m; rotational channels use N*m/rad
// and N*m*s/rad. Failure limits are signed N or N*m, respectively. The coupled
// criterion is sum(alpha * normalized_load^beta) > 1 (strict native boundary).
struct Property {
  double mass_kg=0,isotropic_inertia_kg_m2=0;
  double stiffness[4]{},damping[4]{};
  double failure_negative[4]{},failure_positive[4]{};
  double failure_weight[4]{},failure_exponent[4]{};
};

// Original two-node reference frame. The source's default skew uses global Y
// as the transverse seed, and global X if Y is nearly parallel to the chord.
// Explicit seed axes permit the same construction after a rigid frame change.
struct FrameSeed { Vec3 x{1,0,0},y{0,1,0}; };
struct Reference {
  Vec3 position[2]{};
  Vec3 transverse_axis{};
  double length_m=0;
};
struct EndpointKinematics { Vec3 position{},velocity{},angular_velocity{}; };
struct EndpointWrench { Vec3 force_N{},couple_Nm{}; };
struct Frame {
  Matrix3 axes{},midpoint_axes{}; // Columns are local axes in world coordinates.
  double length_m=0,midpoint_length_m=0;
};
struct History {
  Vec3 transverse_axis{},displacement_m{},rotation_rad{};
  Vec3 local_force_N{},local_couple_Nm{};
  double internal_work_J[4]{};
  double failure_criterion=0;
  bool active=true;
};
struct Evaluation {
  History history{};
  Frame frame{};
  EndpointWrench endpoints[2]{};
  // Native unscaled element dt; caller must apply its declared safety factor
  // and combine with shell/contact/owner admission. No time is advanced here.
  double critical_dt_s=0;
  double translation_stiffness_N_per_m=0,rotation_stiffness_Nm_per_rad=0;
};

struct PropertyInput { std::uint64_t source_property_id=0; Property property{}; };
struct ConnectionInput {
  std::uint64_t source_element_id=0,source_node_id[2]{};
  std::size_t global_node[2]{},property_index=0;
  Vec3 position[2]{};
  FrameSeed seed{};
};
// Explicit source-property contribution. Both endpoint entries carry .5 M and
// .5 J. They are separate from shell mass, physical shell J and added drilling J.
struct EndpointMass {
  std::uint64_t source_element_id=0,source_property_id=0,source_node_id=0;
  std::size_t global_node=0;
  double mass_kg=0,isotropic_inertia_kg_m2=0;
};
struct ModelLimits {
  std::size_t max_connections=LegacyCapacity.connections,max_properties=LegacyCapacity.properties,max_nodes=LegacyCapacity.nodes;
  std::size_t max_host_bytes=4*1024*1024;
  CapacityProfile profile=CapacityProfile::Legacy;
  static constexpr ModelLimits Vehicle() noexcept {
    return {VehicleCapacity.connections,VehicleCapacity.properties,VehicleCapacity.nodes,
            VehicleCapacity.model_host_bytes,CapacityProfile::Vehicle};
  }
};
struct ModelInput {
  std::uint64_t source_instance_id=0;
  std::size_t global_node_count=0;
  const PropertyInput* properties=nullptr;
  std::size_t property_count=0;
  const ConnectionInput* connections=nullptr;
  std::size_t connection_count=0;
  SourceUnits source_units{};
  ModelLimits limits{};
};
} // namespace tl::fea::type25
