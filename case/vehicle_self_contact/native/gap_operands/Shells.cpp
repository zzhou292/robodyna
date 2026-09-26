#include "Internal.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "lib_src/collision/radioss_type25/source_shells/Values.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
double PropertyThickness(const modelio::assembly::Section& section,n::UnitScale units) {
    Require(section.source.keyword=="*SECTION_SHELL" &&
        (section.source_elform==2||section.source_elform==16||section.source_elform==9) && section.cards.size()>=2,
        "Gap property is outside the ordinary source profile");
    const double thickness=modelio::assembly::reader::RequiredMaterialCard(section.cards[1],0);
    Require(std::isfinite(units.length_m)&&units.length_m>0&&std::isfinite(thickness)&&thickness>0&&
        tl::math::SameScalarBits(thickness*units.length_m,section.thickness_m[0]),
        "Gap native property T1 differs from typed declaration");
    return thickness;
}
void Shells(const source::CorrectedNodalSource& corrected,Packed& out) {
    const auto& seed=corrected.pre_correction();
    const auto& references=seed.physical().shell_source().references();
    Require(references.resolution(),"Gap physical shell section resolution missing");
    const auto& rows=references.rows();const auto units=seed.provenance().units;
    bool saw_triangle=false;
    for(const auto& contributor:seed.contributors()) {
        const bool quad=contributor.kind==nodal_seed::ContributorKind::ShellQ4;
        const bool tri=contributor.kind==nodal_seed::ContributorKind::ShellT3;
        if(!quad&&!tri)continue;
        Require(!saw_triangle||tri,"Gap physical shell family order differs");
        saw_triangle=saw_triangle||tri;
        Require(contributor.source_index<rows.size(),"Gap physical shell source index differs");
        const auto& row=rows[contributor.source_index];
        const auto* section=references.resolution()->section(row.part_index);
        Require(row.element_id==contributor.original_id && row.part_id==contributor.part_id &&
            section&&section->id==row.section_id && section->source.keyword=="*SECTION_SHELL" &&
            (section->source_elform==2||section->source_elform==16||section->source_elform==9) &&
            section->cards.size()>=2,"Gap shell/property source identity differs");
        const double thickness=PropertyThickness(*section,units);
        values::PhysicalShell value;
        value.source_element_id=contributor.original_id;
        value.layout=tri?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
        value.property_thickness=thickness;
        // Both zeros are source-proved absence, separate from structural THK.
        value.element_thickness=0.;value.part_contact_thickness=0.;
        const unsigned slots=tri?3:4;
        Require(contributor.slots==slots,"Gap shell source endpoint extent differs");
        for(unsigned k=0;k<slots;++k) {
            Require(contributor.nodes[k]<seed.counts().nodes,"Gap shell node outside shared physical domain");
            value.nodes[k]=contributor.nodes[k];
        }
        if(tri)value.nodes[3]=value.nodes[2];
        CheckMaximumTerm(n::source_shells::detail::HalfGap(value,0));
        ++out.proof.maximum_terms;
        out.bindings.push_back({tri?Family::Triangle:Family::Quad,contributor.original_id,contributor.native_id,
            contributor.part_id,contributor.source_index,out.shells.size()});
        out.shells.push_back(value);
        tri?++out.counts.triangles:++out.counts.quads;
    }
    out.counts.shells=out.shells.size();
    Require(out.counts.shells==seed.counts().shells && out.counts.shells==rows.size(),"Gap shell source coverage incomplete");
}
}
