#pragma once
#include "../FiniteWallContactSource.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
Controls DeclaredControls(n::UnitScale,double friction);
Controls ResolveControls(const WallSource&,const VehicleSource&,Declaration);
struct Component {
    n::source_shells::PhysicalShell shell;
    std::array<double,4> global_coefficients{};
    double primary_coefficient=0, half_gap=0;
};
Component WallComponent(const MaterialDeclaration&,n::UnitScale,std::uint64_t element,
    const std::array<std::uint32_t,4>& nodes);
std::vector<n::source_gaps::PhysicalShell> GapShells(tl::util::ConstView<n::source_gaps::PhysicalShell>,
    std::size_t quads,const n::source_gaps::PhysicalShell& wall,std::size_t byte_cap);
void ScaleSecondary(const double* global,std::size_t nodes,const std::uint32_t* nsv,
    std::size_t count,double scale,double* output);
}
