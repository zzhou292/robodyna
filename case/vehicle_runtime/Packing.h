#pragma once
#include "SourceRoles.h"
#include "lib_src/solvers/FENodalState.h"
namespace crash::cases::vehicle_runtime::detail {
struct OwnerPacking {
    std::vector<double> position, velocity, spin, orientation, mass, inertia, inverse_mass, inverse_inertia;
    std::vector<std::uint8_t> fixed, rotation_fixed, rotation_present;
    tl::fea::HostNodalKinematicsView kinematics() const noexcept;
    tl::fea::NodalDofConfig dofs() const noexcept;
    std::size_t capacity_bytes() const noexcept;
};
std::size_t PackingBytes(std::size_t nodes,std::size_t cap);
OwnerPacking PackOwner(const tl::fea::NodalCoefficientLedger&,const tl::fea::NodalRigidAssemblyBinding&,
                      const SourceRoles&,tl::math::Vec3 velocity,std::size_t cap);
} // namespace crash::cases::vehicle_runtime::detail
