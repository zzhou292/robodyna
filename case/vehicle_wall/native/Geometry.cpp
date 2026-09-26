#include "Internal.h"
#include "modelio/source_assembly/SourceShellReferenceInput.h"
#include "modelio/source_assembly/SourceReferenceMetric.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
#include "lib_src/collision/radioss_type25/CoefficientUnits.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/math/ScalarBits.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::vehicle_wall::native::detail {
namespace n=tlfea::contact::radioss_type25;
namespace shells=n::source_shells;
void Check(const Declaration& d) {
    Require(d.profile==Profile::EnvelopeFixedElasticV1,"Explicit envelope fixed-elastic profile is required");
    const auto& m=d.material;
    Require(m.young_pa==200e9 && m.poisson==.3 && m.density_kg_m3==7860 && m.thickness_m==.001,
            "Wall material differs from the explicit fixed-elastic case declaration");
    Require(d.wall_friction==.6,"Wall friction must remain the separate declared0.6 law operand");
}
std::array<tlfea::contact::Vec3,2> Bounds(const tl::fea::NodalNodeDomain& domain) {
    Require(domain.node_count()!=0,"Vehicle source domain is empty");
    const auto first=domain.nodes()[0].position;
    std::array<tlfea::contact::Vec3,2> bounds{{{first.x,first.y,first.z},{first.x,first.y,first.z}}};
    for(const auto& node:domain.nodes()) {
        const auto x=node.position;
        Require(tl::math::fixed3::Finite(x),"Vehicle source coordinate is nonfinite");
        bounds[0].x=std::min(bounds[0].x,x.x);bounds[0].y=std::min(bounds[0].y,x.y);bounds[0].z=std::min(bounds[0].z,x.z);
        bounds[1].x=std::max(bounds[1].x,x.x);bounds[1].y=std::max(bounds[1].y,x.y);bounds[1].z=std::max(bounds[1].z,x.z);
    }
    return bounds;
}
Geometry BuildGeometry(const std::array<tlfea::contact::Vec3,2>& bounds,const Declaration& d,
        const AllocatedIds& ids,const source::Units& units) {
    Check(d);source::CheckUnits(units);
    const n::UnitScale native_units{units.length_to_m,units.mass_to_kg,units.time_to_s};
    n::coefficient_detail::UnitFactors factors;
    Require(n::coefficient_detail::Make(native_units,factors),"Unrepresentable native wall working units");
    const double pressure=factors.pressure;
    Require(std::isfinite(pressure)&&pressure>0,"Unrepresentable native pressure unit");
    shells::PhysicalShell wall;
    wall.source_element_id=ids.shell;wall.layout=n::ShellLayout::Quad4;
    for(unsigned i=0;i<4;++i)wall.nodes[i]=i;
    wall.young=d.material.young_pa/pressure;
    wall.structural_thickness=d.material.thickness_m/units.length_to_m;
    wall.property_thickness=wall.structural_thickness;
    // Explicit absent overrides in this newly declared ordinary wall component.
    // No final vehicle NSV/main population is fabricated by this value call.
    const std::uint32_t primary=0;
    shells::Input input;
    input.profile={shells::Population::OrdinaryShellsOnly,1,0,1,1,0,0,1.,1.,1e30,1e30};
    input.node_count=4;input.shells=&wall;input.shell_count=1;
    input.primary_shells=&primary;input.primary_count=1;
    shells::Limits limits;limits.nodes=4;limits.shells=1;limits.primaries=1;limits.secondaries=4;
    shells::Forecast forecast;
    Require(shells::Preflight(input,limits,forecast).status==shells::Status::Ok,"Ordinary wall gap/primary forecast rejected");
    tl::util::HostArena scratch;
    Require(scratch.Initialize(forecast.scratch_bytes),"Wall component scratch allocation failed");
    shells::NodeFields fields[4];double coefficient=0;
    Require(shells::Build(input,limits,scratch.data(),scratch.bytes(),{fields,4,&coefficient,1,nullptr,0}).status==shells::Status::Ok,
            "Original ordinary wall gap/primary coefficient producer rejected");
    Geometry out;
    out.native_half_gap=fields[0].main_gap;
    for(const auto& field:fields)
        Require(tl::math::SameScalarBits(field.main_gap,out.native_half_gap),"Declared constant wall gap is not uniform");
    out.component_primary_stiffness_native=coefficient; // Original E*h; never area compensation.
    out.reference_offset_m=out.native_half_gap*units.length_to_m;
    Require(std::isfinite(out.reference_offset_m)&&out.reference_offset_m>0,"Wall gap offset is not representable");
    Settings settings;settings.mesh_profile=WallMeshProfile::EnvelopeRectangleV1;
    settings.requested_duration_s=d.requested_duration_s;settings.leading_gap_m=d.leading_gap_m;
    settings.transverse_margin_m=d.transverse_margin_m;settings.exposed_clearance_m=d.exposed_clearance_m;
    settings.wall_binding_id=d.binding_id;
    // Existing public placement/coverage helpers already precede runtime owner
    // creation; no duplicate geometry implementation or area-penalty call.
    out.placement=Place(bounds,settings);
    const auto envelope=EnvelopeWall::Prepare(out.placement,settings);
    out.envelope=envelope.extent();
    out.reference_plane_m=out.placement.represented_wall_x_m+out.reference_offset_m;
    Require(std::isfinite(out.reference_plane_m)&&tl::math::SameScalarBits(
        out.reference_plane_m-out.reference_offset_m,out.placement.represented_wall_x_m),
        "Reference-plane offset cannot preserve the chosen front contact plane");
    const auto view=envelope.view();
    modelio::assembly::SourceReferenceNode nodes[4];
    for(unsigned i=0;i<4;++i) {
        const auto x=view.vertices[i].position;
        out.reference_m[i]={out.reference_plane_m,x.y,x.z};
        out.display_feature_ids[i]=view.vertices[i].source_node_id;
        auto& native=out.reference_native[i];
        native={out.reference_m[i].x/units.length_to_m,out.reference_m[i].y/units.length_to_m,
                out.reference_m[i].z/units.length_to_m};
        Require(tl::math::fixed3::Finite(native)&&
            tl::math::SameScalarBits(native.x*units.length_to_m,out.reference_m[i].x)&&
            tl::math::SameScalarBits(native.y*units.length_to_m,out.reference_m[i].y)&&
            tl::math::SameScalarBits(native.z*units.length_to_m,out.reference_m[i].z),
            "Native wall coordinates do not round-trip to the selected physical geometry");
        nodes[i]={ids.nodes[i],out.reference_m[i]};
    }
    modelio::assembly::Material material;
    material.id=ids.material;material.young_pa=d.material.young_pa;
    material.poisson_ratio=d.material.poisson;material.density_kg_m3=d.material.density_kg_m3;
    material.law=modelio::assembly::MaterialLaw::LayeredLaw1;
    modelio::assembly::Section section;section.id=ids.section;section.through_thickness_points=3;
    section.thickness_m.fill(d.material.thickness_m);
    auto reference=modelio::assembly::PackShellReference<tl::fea::qeph::ReferenceInput>(nodes,material,section);
    const auto metric=modelio::assembly::QephReferenceMetric::Resolve(
        modelio::assembly::QephMetricProfile::AuthenticatedSourceLength,
        {units.mass_to_kg,units.length_to_m,units.time_to_s});
    out.reference_input=modelio::assembly::WithQephMetric(reference,metric);
    Require(tl::fea::qeph::InitializeReference(out.reference_input,out.reference)==tl::fea::qeph::Status::kSuccess,
            "Declared wall reference rejected by the original QEPH source producer");
    out.native_working_length_m=metric.working_length_m();
    for(double mass:out.reference.nodal_mass)out.wall_mass_kg+=mass;
    Require(std::isfinite(out.wall_mass_kg)&&out.wall_mass_kg>0,"Declared wall mass is not finite positive");
    return out;
}
} // namespace crash::cases::vehicle_wall::native::detail
