#pragma once
#include "SourceAssemblyObservation.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <array>
#include <cmath>
#include <limits>

namespace crash::cases::source_assembly_observation::detail {
// Matches the bounded owner recurrence. No owner-private storage is accessed.
inline constexpr std::size_t MaxGroups = 64, MaxMembers = 256;
inline constexpr long double Epsilon = std::numeric_limits<double>::epsilon();
using Membership = std::array<bool, fe::MaxNodalStateNodes>;
inline Report Success() noexcept { return {Status::Ok, "Observation complete"}; }
inline tl::math::Vec3 Vector(const double* values, std::size_t node) noexcept {
    return {values[3*node], values[3*node+1], values[3*node+2]};
}
inline rigid::MemberMotion Motion(fe::HostNodalKinematicsView v, std::size_t n) noexcept {
    return {Vector(v.velocity_xyz,n), Vector(v.angular_velocity_xyz,n)};
}
inline bool Scope(const SourceAssemblyBindings& b, const fe::NodalRigidGroupInfo& s) noexcept {
    const auto* m=b.rigid_groups();
    return m && s.source_instance_id==b.source_instance_id() &&
        s.group_count==m->group_count() && s.member_count==m->member_count();
}
inline bool Range(const void* data, std::size_t bytes, const void* out, std::size_t out_bytes) noexcept {
    return data && fe::trial_identity::Disjoint(data,bytes,out,out_bytes);
}
Report CheckFrame(const SourceAssemblyBindings&, fe::HostNodalKinematicsView,
                  const fe::NodalRigidGroupSnapshot*, std::size_t,
                  const void* output, std::size_t output_bytes) noexcept;
Report AcceptedPhase(const fe::NodalStamp&, rigid::ObservationPhase&) noexcept;
Report ObserveKinetic(const SourceAssemblyBindings&, fe::HostNodalKinematicsView,
                      const fe::NodalRigidGroupSnapshot*, rigid::ObservationPhase,
                      const fe::ShellBatchKinetic&, KineticSummary&, Membership&) noexcept;
Report Convert(const rigid::ObservationReport&, std::size_t group,
               const fe::NodalRigidGroupModel&) noexcept;
} // namespace crash::cases::source_assembly_observation::detail
