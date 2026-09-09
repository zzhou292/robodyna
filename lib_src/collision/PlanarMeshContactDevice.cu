#include "PlanarMeshContactStorage.h"

namespace tlfea::contact {
namespace {
namespace fea = tl::fea;
using PStatus = PlanarContactStatus;
using planar_detail::Control;

__device__ void Fail(Control* c, const fea::NodalAssemblyView& view,
                     PStatus status, Status assembly_status, std::uint32_t sample = UINT32_MAX) {
  c->status = status; c->sample = sample; c->diagnostics.valid = false;
  fea::RecordNodalAssemblyFailure(view,assembly_status);
}
__global__ void RejectAssembly(fea::NodalAssemblyView view) {
  fea::RecordNodalAssemblyFailure(view,Status::kInvalidArgument);
}

__device__ PStatus ValidateMotion(const fea::DeviceNodalKinematicsView& view,
                                  const PlanarSurfaceNode* nodes, std::uint32_t count,
                                  const planar_detail::SurfacePoint* points, std::uint32_t point_count,
                                  double wall_x, double maximum_depth, std::uint32_t* bad_node) {
  for (std::uint32_t i = 0; i < count; ++i) {
    const auto global = nodes[i].global_node; *bad_node = global;
    const auto offset = 3*global;
    const Vec3 x{view.position_xyz[offset],view.position_xyz[offset+1],view.position_xyz[offset+2]};
    const Vec3 v{view.velocity_xyz[offset],view.velocity_xyz[offset+1],view.velocity_xyz[offset+2]};
    if (!IsFinite(x) || !IsFinite(v)) return PStatus::InvalidOutput;
    if (x.y != nodes[i].reference_position.y || x.z != nodes[i].reference_position.z || v.y != 0 || v.z != 0)
      return PStatus::UnsupportedMotion;
    if (view.angular_velocity_xyz)
      for (unsigned axis = 0; axis < 3; ++axis)
        if (view.angular_velocity_xyz[offset+axis] != 0) return PStatus::UnsupportedMotion;
    bool covered = false;
    for (std::uint32_t p = 0; p < point_count; ++p)
      if (points[p].covered)
        for (int j = 0; j < 3; ++j) covered = covered || points[p].local_nodes[j] == i;
    if (covered) {
      const double depth = x.x-wall_x;
      if (!IsFinite(depth)) return PStatus::InvalidOutput;
      if (depth > maximum_depth) return PStatus::UnsupportedMotion;
    }
  }
  *bad_node = UINT32_MAX; return PStatus::Ok;
}

__global__ void EvaluateBatch(PlanarContactConfig config, const planar_detail::WallFace* wall,
                              std::uint32_t wall_count, const PlanarSurfaceNode* nodes,
                              std::uint32_t node_count, const planar_detail::SurfacePoint* points,
                              std::uint32_t point_count, double wall_x, double tolerance,
                              fea::NodalAssemblyView view, Control* c) {
  // One writer matches the existing serialized row-bound/scatter contract.
  *c = {};
  auto& d = c->diagnostics;
  d.owner_id = config.owner_id; d.base_epoch = view.accepted.base_epoch; d.attempt = view.attempt;
  d.sample_count = point_count;
  if (!view.bounds->valid || !view.bounds->initialized || view.bounds->sealed ||
      view.bounds->base_epoch != d.base_epoch || view.bounds->attempt != d.attempt ||
      view.result->base_epoch != d.base_epoch || view.result->attempt != d.attempt ||
      view.result->status != Status::kOk) {
    Fail(c,view,PStatus::StaleAttempt,Status::kStaleTrial); return;
  }
  std::uint32_t bad_node;
  const auto motion = ValidateMotion(view.accepted,nodes,node_count,points,point_count,wall_x,config.max_penetration_m,&bad_node);
  if (motion != PStatus::Ok) {
    Fail(c,view,motion,Status::kInvalidArgument,bad_node); return;
  }
  const VectorView positions{view.accepted.position_xyz,config.global_node_count,3,1};
  const VectorView velocities{view.accepted.velocity_xyz,config.global_node_count,3,1};
  for (std::uint32_t i = 0; i < point_count; ++i) {
    const auto& source = points[i]; auto& sample = d.samples[i];
    sample.surface_triangle_id = source.triangle.feature_id;
    sample.reference_area = source.area; sample.stiffness = source.stiffness;
    const LinearTriangleSurfaceView surface{positions,velocities,view.mass.inverse_mass,&source.triangle,1};
    const LinearTrianglePoint location{0,{1.0/3,1.0/3,1.0/3}};
    LinearPointKinematics kinematics;
    auto status = EvaluateLinearPoint(surface,location,&kinematics);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
    sample.surface_point = kinematics.position;
    sample.wall_point = {wall_x,kinematics.position.y,kinematics.position.z};
    sample.gap = wall_x-kinematics.position.x;
    sample.normal_velocity = -kinematics.velocity.x;
    if (!IsFinite(sample.gap) || !IsFinite(sample.wall_point)) {
      Fail(c,view,PStatus::InvalidOutput,Status::kNonFiniteResult,i); return;
    }
    std::uint32_t owner; TrianglePointGeometry closest;
    status = planar_detail::FindOwner(sample.wall_point,wall,wall_count,tolerance,&owner,&closest);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
    sample.covered = owner != UINT32_MAX;
    if (sample.covered != source.covered) {
      Fail(c,view,PStatus::UnsupportedMotion,Status::kInvalidArgument,i); return;
    }
    if (!sample.covered) continue;
    ++d.covered_count;
    sample.wall_triangle_id = wall[owner].geometry.face_id;
    sample.wall_source_quad_id = wall[owner].source_quad_id;
    sample.wall_assembled_source_quad_id = wall[owner].assembled_source_quad_id;
    sample.wall_feature = closest.feature;
    NormalJacobian jacobian;
    status = BuildLinearTriangleNormalJacobian(view.mass,source.triangle,location.weights,
                                               nullptr,nullptr,{-1,0,0},view.attempt,&jacobian);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
    NormalContactResponse response;
    status = EvaluateNormalContact({source.stiffness,0,.8},
        {sample.gap,sample.normal_velocity,jacobian.inverse_effective_mass},&response);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
    // Conservative ALL-covered-springs bound, including current positive gaps.
    status = fea::stability::MakeRankOneContribution(jacobian,source.stiffness,0,&c->contributions[i]);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
    sample.active = response.active; if (sample.active) ++d.active_count;
    sample.force_on_surface = {-response.force,0,0};
    sample.elastic_energy = response.elastic_energy;
    sample.surface_power = response.force*sample.normal_velocity;
    const auto reaction = Scale(sample.force_on_surface,-1);
    const auto moment = geometry_detail::Cross(sample.wall_point,reaction);
    d.force_on_surface = Add(d.force_on_surface,sample.force_on_surface);
    d.wall_reaction = Add(d.wall_reaction,reaction);
    d.wall_moment = Add(d.wall_moment,moment);
    d.elastic_energy += sample.elastic_energy; d.surface_power += sample.surface_power;
    if (!IsFinite(sample.surface_power) || !IsFinite(d.force_on_surface) || !IsFinite(d.wall_reaction) ||
        !IsFinite(d.wall_moment) || !IsFinite(d.elastic_energy) || !IsFinite(d.surface_power)) {
      Fail(c,view,PStatus::InvalidOutput,Status::kNonFiniteResult,i); return;
    }
    TriangleNodalForces force;
    status = ProjectLinearPointForce(surface,location,sample.force_on_surface,&force);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
    for (int j = 0; j < 3; ++j) {
      auto& staged = c->staged_force[source.local_nodes[j]];
      staged = Add(staged,force.forces[j]);
      if (!IsFinite(staged)) { Fail(c,view,PStatus::InvalidOutput,Status::kNonFiniteResult,i); return; }
    }
  }
  // Validate additive force writes before publishing any of this batch's force.
  for (std::uint32_t i = 0; i < node_count; ++i) {
    const auto n = nodes[i].global_node;
    const Vec3 sum{view.forces.force_x[n]+c->staged_force[i].x,
                   view.forces.force_y[n]+c->staged_force[i].y,
                   view.forces.force_z[n]+c->staged_force[i].z};
    if (!IsFinite(sum)) { Fail(c,view,PStatus::InvalidOutput,Status::kNonFiniteResult); return; }
    c->staged_force[i] = sum;
  }
  for (std::uint32_t i = 0; i < point_count; ++i) if (points[i].covered) {
    const auto status = fea::stability::AccumulateRows(view.bounds,c->contributions[i]);
    if (status != Status::kOk) { Fail(c,view,PStatus::InvalidOutput,status,i); return; }
  }
  for (std::uint32_t i = 0; i < node_count; ++i) {
    const auto n = nodes[i].global_node;
    view.forces.force_x[n] = c->staged_force[i].x;
    view.forces.force_y[n] = c->staged_force[i].y;
    view.forces.force_z[n] = c->staged_force[i].z;
  }
  d.valid = true;
}

__global__ void ValidateCandidate(fea::NodalPreparedView prepared, const PlanarSurfaceNode* nodes,
                                  std::uint32_t count, const planar_detail::SurfacePoint* points,
                                  std::uint32_t point_count, double wall_x, double max_depth, Control* c) {
  std::uint32_t bad_node;
  c->status = ValidateMotion(prepared.kinematics,nodes,count,points,point_count,wall_x,max_depth,&bad_node);
  c->sample = bad_node;
  if (c->status != PStatus::Ok) c->diagnostics.valid = false;
}

const char* Message(PStatus status) {
  switch (status) {
    case PStatus::Ok: return "Planar contact trial evaluated";
    case PStatus::UnsupportedMotion: return "Normal-only motion or penetration bound violated";
    case PStatus::StaleAttempt: return "Assembly provenance/phase is stale";
    default: return "Planar contact arithmetic or assembly failed";
  }
}
bool AssemblyPointers(const fea::NodalAssemblyView& view) {
  return view.accepted.position_xyz && view.accepted.velocity_xyz && view.mass.inverse_mass && view.mass.fixed &&
         view.forces.force_x && view.forces.force_y && view.forces.force_z &&
         view.forces.couple_x && view.forces.couple_y && view.forces.couple_z && view.bounds && view.result;
}
}  // namespace

PlanarContactReport PlanarMeshContact::Evaluate(const fea::NodalAssemblyView& view) {
  if (!impl_) return {PStatus::NotInitialized,"Contact batch is not initialized"};
  auto& s = *impl_; s.host_control.diagnostics.valid = false;
  if (!s.usable) return {PStatus::DeviceFailure,"Contact batch is poisoned"};
  auto check = [&](cudaError_t error) {
    if (error == cudaSuccess) return true;
    s.usable = false; s.host_control.diagnostics.valid = false; return false;
  };
  auto reject = [&](PStatus status, const char* message) -> PlanarContactReport {
    // With a valid borrowed assembly, make omission impossible to overlook at
    // SealAssembly. Missing/invalid borrowed storage still requires caller discard.
    if (view.result && view.bounds) {
      RejectAssembly<<<1,1,0,view.stream>>>(view);
      const auto launch = cudaGetLastError();
      if (!check(launch)) return {PStatus::DeviceFailure,cudaGetErrorString(launch)};
      const auto sync = cudaStreamSynchronize(view.stream);
      if (!check(sync)) return {PStatus::DeviceFailure,cudaGetErrorString(sync)};
    }
    return {status,message};
  };
  if (view.owner_id != s.config.owner_id) return reject(PStatus::WrongOwner,"Assembly belongs to another nodal owner");
  if (!fea::IsCollocatedNodalTiming(view.temporal_scheme,view.velocity_phase) ||
      !AssemblyPointers(view) || !view.attempt || view.accepted.node_count != s.config.global_node_count ||
      view.mass.node_count != s.config.global_node_count || view.forces.node_count != s.config.global_node_count ||
      view.mass.model != TranslationMassModel::kIsotropicLumped)
    return reject(PStatus::InvalidInput,"Invalid borrowed nodal assembly");
  const auto epoch = view.accepted.base_epoch;
  if (view.mass.base_epoch != epoch || view.forces.base_epoch != epoch ||
      (s.attempted && (epoch < s.last_epoch || (epoch == s.last_epoch && view.attempt <= s.last_attempt))))
    return reject(PStatus::StaleAttempt,"Duplicate or stale contact contribution");
  s.last_epoch = epoch; s.last_attempt = view.attempt; s.attempted = true;
  EvaluateBatch<<<1,1,0,view.stream>>>(s.config,s.wall,s.wall_count,s.nodes,s.node_count,
      s.points,s.point_count,s.wall_x,s.geometry_tolerance,view,s.control);
  auto error = cudaGetLastError();
  if (!check(error)) return {PStatus::DeviceFailure,cudaGetErrorString(error)};
  error = cudaMemcpyAsync(&s.host_control,s.control,sizeof(Control),cudaMemcpyDeviceToHost,view.stream);
  if (!check(error)) return {PStatus::DeviceFailure,cudaGetErrorString(error)};
  error = cudaStreamSynchronize(view.stream);
  if (!check(error)) return {PStatus::DeviceFailure,cudaGetErrorString(error)};
  return {s.host_control.status,Message(s.host_control.status),s.host_control.sample};
}

PlanarContactReport PlanarMeshContact::ValidatePrepared(const fea::NodalPreparedView& prepared) {
  if (!impl_) return {PStatus::NotInitialized,"Contact batch is not initialized"};
  auto& s = *impl_;
  auto reject = [&](PStatus status, const char* message) -> PlanarContactReport {
    s.host_control.diagnostics.valid = false; return {status,message};
  };
  if (!s.usable) return reject(PStatus::DeviceFailure,"Contact batch is poisoned");
  const auto& d = s.host_control.diagnostics;
  if (prepared.owner_id != s.config.owner_id) return reject(PStatus::WrongOwner,"Prepared state belongs to another owner");
  if (!d.valid || prepared.kinematics.base_epoch != d.base_epoch || prepared.attempt != d.attempt)
    return reject(PStatus::StaleAttempt,"Prepared state does not match a successful contact evaluation");
  if (!fea::IsCollocatedNodalTiming(prepared.temporal_scheme,prepared.velocity_phase) ||
      !fea::IsCollocatedNodalTiming(prepared.temporal_scheme,prepared.base_velocity_phase) ||
      !prepared.kinematics.position_xyz || !prepared.kinematics.velocity_xyz ||
      prepared.kinematics.node_count != s.config.global_node_count || !IsFinite(prepared.proposed_time) || prepared.proposed_time <= 0)
    return reject(PStatus::InvalidInput,"Invalid borrowed prepared state");
  ValidateCandidate<<<1,1,0,prepared.stream>>>(prepared,s.nodes,s.node_count,s.points,s.point_count,s.wall_x,s.config.max_penetration_m,s.control);
  auto error = cudaGetLastError();
  if (error == cudaSuccess) error = cudaMemcpyAsync(&s.host_control,s.control,sizeof(Control),cudaMemcpyDeviceToHost,prepared.stream);
  if (error == cudaSuccess) error = cudaStreamSynchronize(prepared.stream);
  if (error != cudaSuccess) { s.usable = false; return reject(PStatus::DeviceFailure,cudaGetErrorString(error)); }
  return {s.host_control.status,Message(s.host_control.status),s.host_control.sample};
}
}  // namespace tlfea::contact
