#include "Values.h"
#include "output/ArtifactIO.h"
#include "lib_src/collision/radioss_type25/CoefficientUnits.h"
#include <cmath>
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
Component WallComponent(const MaterialDeclaration& material,n::UnitScale units,std::uint64_t element,
    const std::array<std::uint32_t,4>& nodes) {
    n::coefficient_detail::UnitFactors factors;
    output::Require(element && n::coefficient_detail::Make(units,factors),"Invalid native wall component units/identity");
    Component out;
    auto& shell=out.shell;
    shell.source_element_id=element;shell.layout=n::ShellLayout::Quad4;
    shell.young=material.young_pa/factors.pressure;
    shell.structural_thickness=material.thickness_m/units.length_m;
    shell.property_thickness=shell.structural_thickness;
    for(unsigned k=0;k<4;++k)shell.nodes[k]=k;
    n::source_shells::Profile profile;
    profile.population=n::source_shells::Population::OrdinaryShellsOnly;
    profile.property_type=1;profile.input_thickness_mode=0;profile.level=1;profile.gap_mode=1;
    profile.free_edge_gap=0;profile.contact_thickness_update=0;profile.stiffness_scale=1;profile.gap_scale=1;
    profile.maximum_secondary_gap=n::native_constant::ep20*n::native_constant::ep10;
    profile.maximum_main_gap=profile.maximum_secondary_gap;
    const std::uint32_t primary=0;
    n::source_shells::Input input{profile,4,&shell,1,&primary,1,nullptr,0};
    n::source_shells::Limits limits{4,1,1,4,1u<<20};
    n::source_shells::Forecast forecast;
    output::Require(n::source_shells::Preflight(input,limits,forecast).status==n::source_shells::Status::Ok,
        "Wall component coefficient forecast rejected");
    tl::util::HostArena scratch;
    output::Require(scratch.Initialize(forecast.scratch_bytes),"Wall component coefficient allocation failed");
    n::source_shells::NodeFields fields[4];
    output::Require(n::source_shells::Build(input,limits,scratch.data(),scratch.bytes(),
        {fields,4,&out.primary_coefficient,1,nullptr,0}).status==n::source_shells::Status::Ok,
        "Native wall nodal/main coefficient producer rejected");
    out.half_gap=fields[0].main_gap;
    for(unsigned k=0;k<4;++k) {
        out.global_coefficients[k]=fields[k].stiffness;
        shell.nodes[k]=nodes[k];
    }
    return out;
}
void ScaleSecondary(const double* global,std::size_t nodes,const std::uint32_t* nsv,
    std::size_t count,double scale,double* output) {
    output::Require(global && nsv && output && nodes && count,"Incomplete wall secondary source");
    for(std::size_t i=0;i<count;++i) {
        output::Require(nsv[i]<nodes,"Wall secondary node is outside complete domain");
        n::NativeScalarCoefficient value;
        // I25STI3 initializes this declared INACTI5 profile's STFNS to ONE.
        output::Require(n::EvaluateNativeSecondaryCoefficient({1.,global[nsv[i]],scale},&value)==n::CoefficientStatus::Ok,
            "Native wall secondary coefficient producer rejected");
        output[i]=value.value;
    }
}
}
