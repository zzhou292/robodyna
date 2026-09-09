#pragma once

// Allocation-free scalar measurements over borrowed owner state. Case-specific
// tolerances/admission remain outside the batch; no dynamics is implemented.
#include "ReissnerShellBatchStorage.h"
#include <cfloat>

namespace tl::fea::reissner::batch_detail {

TL_SURFACE_HD inline Vec3 ReadVector(const double* values, std::size_t node) {
  return {values[3*node], values[3*node+1], values[3*node+2]};
}
TL_SURFACE_HD inline Quaternion ReadRotation(const double* values, std::size_t node) {
  return {values[4*node], values[4*node+1], values[4*node+2], values[4*node+3]};
}
TL_SURFACE_HD inline double Length(Vec3 vector) {
  return ::hypot(::hypot(vector.x, vector.y), vector.z);
}
TL_SURFACE_HD inline double Maximum(double a, double b) { return a > b ? a : b; }
TL_SURFACE_HD inline double Minimum(double a, double b) { return a < b ? a : b; }

TL_SURFACE_HD inline double RotationAngle(Quaternion a, Quaternion b) {
  const auto difference = detail::Product(a, Quaternion{b.w, -b.x, -b.y, -b.z});
  return 2 * ::atan2(Length({difference.x, difference.y, difference.z}), ::fabs(difference.w));
}

TL_SURFACE_HD inline ShellBatchDiagnostics BeginDiagnostics(
    const Model& model, std::uint64_t epoch, std::uint64_t attempt, ShellBatchPhase phase) {
  ShellBatchDiagnostics value;
  value.owner_id = model.config.owner.owner_id;
  value.configuration_id = model.config.configuration_id;
  value.base_epoch = epoch; value.attempt = attempt; value.phase = phase;
  value.minimum_signed_area_ratio = DBL_MAX;
  value.minimum_area_norm_ratio = DBL_MAX;
  value.minimum_display_triangle_area_ratio = DBL_MAX;
  return value;
}

TL_SURFACE_HD inline bool MeasureArea(Vec3 current, Vec3 reference, ShellBatchDiagnostics& value) {
  const double reference_norm = Length(reference), current_norm = Length(current);
  if (!detail::Finite(reference_norm) || reference_norm <= 0 || !detail::Finite(current_norm)) return false;
  const double norm_ratio = current_norm / reference_norm;
  const double signed_ratio = detail::Dot(current, detail::Scale(reference, 1 / reference_norm)) / reference_norm;
  if (!detail::Finite(norm_ratio) || !detail::Finite(signed_ratio)) return false;
  value.minimum_signed_area_ratio = Minimum(value.minimum_signed_area_ratio, signed_ratio);
  value.minimum_area_norm_ratio = Minimum(value.minimum_area_norm_ratio, norm_ratio);
  value.maximum_area_norm_ratio = Maximum(value.maximum_area_norm_ratio, norm_ratio);
  return true;
}

TL_SURFACE_HD inline bool MeasureElement(
    const Model& model, unsigned e, const DeviceNodalKinematicsView& state,
    ShellResult& force, Control& control) {
  const auto& element = model.element[e];
  ShellConfiguration configuration;
  Vec3 velocity[4], omega[4], director[4];
  Quaternion physical[4];
  for (unsigned n = 0; n < 4; ++n) {
    const auto node = element.nodes[n];
    configuration.position[n] = ReadVector(state.position_xyz, node);
    configuration.rotation[n] = ReadRotation(state.orientation_wxyz, node);
    velocity[n] = ReadVector(state.velocity_xyz, node);
    omega[n] = ReadVector(state.angular_velocity_xyz, node);
    physical[n] = detail::Product(configuration.rotation[n], element.reference.node_frame_offset[n]);
    director[n] = detail::Product(detail::Rotation(physical[n]), Vec3{0, 0, 1});
  }
  const auto status = ComputeShellForce(element.reference, element.section, configuration, force);
  if (status != ShellStatus::kSuccess) {
    control.status = ShellBatchStatus::kElementFailure; control.element = e; control.element_status = status;
    return false;
  }
  ShellKineticEnergy kinetic;
  if (ComputeShellKineticEnergy(model.element_mass[e], director, velocity, omega, kinetic) != ShellMassStatus::kSuccess) {
    control.status = ShellBatchStatus::kNonfiniteResult; control.element = e; return false;
  }
  auto& value = control.diagnostics;
  value.elastic_energy += force.energy; value.bending_energy += force.bending_energy;
  value.kinetic_translation += kinetic.translation;
  value.kinetic_physical_rotation += kinetic.physical_rotation;
  value.kinetic_artificial_drilling += kinetic.artificial_drilling;
  for (unsigned n = 0; n < 4; ++n) {
    const auto initial_physical = detail::Product(element.reference.initial_rotation[n], element.reference.node_frame_offset[n]);
    const double displacement = Length(detail::Subtract(configuration.position[n], element.reference.initial_position[n]));
    const double angle = RotationAngle(physical[n], initial_physical);
    if (!detail::Finite(displacement) || !detail::Finite(angle)) {
      control.status = ShellBatchStatus::kNonfiniteResult; control.element = e; control.node = element.nodes[n]; return false;
    }
    value.maximum_displacement = Maximum(value.maximum_displacement, displacement);
    value.maximum_director_departure = Maximum(value.maximum_director_departure, angle);
    for (unsigned other = 0; other < n; ++other)
      value.maximum_pair_angle = Maximum(value.maximum_pair_angle, RotationAngle(physical[n], physical[other]));
  }
  for (unsigned p = 0; p < 4; ++p) {
    for (unsigned c = 0; c < 6; ++c) {
      value.maximum_membrane_strain = Maximum(value.maximum_membrane_strain, ::fabs(force.strain[p][c]));
      value.maximum_thickness_curvature = Maximum(value.maximum_thickness_curvature,
                                                  element.section.thickness * ::fabs(force.strain[p][6+c]));
    }
    const auto& point = element.reference.gauss[p];
    const auto current = detail::Cross(shell_detail::InterpolateGradient(configuration.position, point, 0),
                                      shell_detail::InterpolateGradient(configuration.position, point, 1));
    const auto initial = detail::Cross(shell_detail::InterpolateGradient(element.reference.initial_position, point, 0),
                                      shell_detail::InterpolateGradient(element.reference.initial_position, point, 1));
    if (!MeasureArea(current, initial, value)) {
      control.status = ShellBatchStatus::kInvalidGeometry; control.element = e; return false;
    }
  }
  constexpr unsigned triangles[2][3] = {{0, 1, 2}, {0, 2, 3}};
  for (const auto& triangle : triangles) {
    const auto current = detail::Cross(
        detail::Subtract(configuration.position[triangle[1]], configuration.position[triangle[0]]),
        detail::Subtract(configuration.position[triangle[2]], configuration.position[triangle[0]]));
    const auto& x0 = element.reference.initial_position;
    const auto initial = detail::Cross(detail::Subtract(x0[triangle[1]], x0[triangle[0]]),
                                      detail::Subtract(x0[triangle[2]], x0[triangle[0]]));
    const double reference_area = Length(initial), current_area = Length(current);
    const double ratio = current_area / reference_area;
    if (!detail::Finite(reference_area) || reference_area <= 0 || !detail::Finite(ratio)) {
      control.status = ShellBatchStatus::kInvalidGeometry; control.element = e; return false;
    }
    value.minimum_display_triangle_area_ratio = Minimum(value.minimum_display_triangle_area_ratio, ratio);
  }
  const double scalars[] = {value.elastic_energy, value.bending_energy, value.kinetic_translation,
      value.kinetic_physical_rotation, value.kinetic_artificial_drilling, value.maximum_pair_angle,
      value.maximum_membrane_strain, value.maximum_thickness_curvature};
  for (double scalar : scalars)
    if (!detail::Finite(scalar)) { control.status = ShellBatchStatus::kNonfiniteResult; control.element = e; return false; }
  return true;
}

TL_SURFACE_HD inline bool MeasureInterval(Storage& storage, const NodalPreparedView& prepared) {
  auto& control = storage.control;
  auto& value = control.diagnostics;
  const auto& base = storage.base;
  value.base_elastic_energy = base.elastic_energy;
  value.base_kinetic_energy = base.kinetic_translation + base.kinetic_physical_rotation + base.kinetic_artificial_drilling;
  value.elastic_energy_increment = value.elastic_energy - value.base_elastic_energy;
  value.kinetic_energy_increment = value.kinetic_translation + value.kinetic_physical_rotation +
                                    value.kinetic_artificial_drilling - value.base_kinetic_energy;
  const auto n = storage.model.config.owner.node_count;
  const double h = storage.model.config.owner.fixed_dt;
  for (std::size_t node = 0; node < n; ++node) {
    const auto& old_state = prepared.base_kinematics;
    const auto& new_state = prepared.kinematics;
    const auto displacement = detail::Subtract(ReadVector(new_state.position_xyz, node), ReadVector(old_state.position_xyz, node));
    const auto mean_velocity = detail::Scale(detail::Add(ReadVector(old_state.velocity_xyz, node), ReadVector(new_state.velocity_xyz, node)), .5);
    const auto mean_omega = detail::Scale(detail::Add(ReadVector(old_state.angular_velocity_xyz, node), ReadVector(new_state.angular_velocity_xyz, node)), .5);
    const auto relative = detail::Product(detail::Rotation(ReadRotation(new_state.orientation_wxyz, node)),
                                          detail::Transpose(detail::Rotation(ReadRotation(old_state.orientation_wxyz, node))));
    Vec3 spin_increment;
    if (ComputeRotationVector(relative, spin_increment) != Status::kSuccess) {
      control.status = ShellBatchStatus::kNonfiniteResult; control.node = node; return false;
    }
    const Vec3 force{storage.force[node], storage.force[n+node], storage.force[2*n+node]};
    const Vec3 couple{storage.force[3*n+node], storage.force[4*n+node], storage.force[5*n+node]};
    value.kinetic_midpoint_work += h * (detail::Dot(force, mean_velocity) + detail::Dot(couple, mean_omega));
    value.force_coordinate_work += detail::Dot(force, displacement) + detail::Dot(couple, spin_increment);
    value.mass_weighted_increment_squared += storage.model.nodal_mass[node] * detail::Dot(displacement, displacement) +
                                             storage.model.nodal_inertia[node] * detail::Dot(spin_increment, spin_increment);
  }
  value.kinetic_work_residual = value.kinetic_energy_increment - value.kinetic_midpoint_work;
  value.conservative_force_coordinate_defect = value.elastic_energy_increment + value.force_coordinate_work;
  const double scalars[] = {value.base_elastic_energy, value.base_kinetic_energy, value.elastic_energy_increment,
      value.kinetic_energy_increment, value.kinetic_midpoint_work, value.force_coordinate_work,
      value.mass_weighted_increment_squared, value.kinetic_work_residual, value.conservative_force_coordinate_defect};
  for (double scalar : scalars)
    if (!detail::Finite(scalar)) { control.status = ShellBatchStatus::kNonfiniteResult; return false; }
  return true;
}

}  // namespace tl::fea::reissner::batch_detail
