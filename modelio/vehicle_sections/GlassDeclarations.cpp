#include "GlassDeclarations.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include "modelio/vehicle_source/SourceCards.h"
#include <cmath>

namespace crash::modelio::vehicle::resolution {
using namespace assembly::reader;
namespace {
void Names(const assembly::DeclarationCard& card, std::initializer_list<const char*> names) {
    Require(card.names.size() == names.size(), "Glass card field count changed");
    std::size_t i = 0;
    for (const auto* name : names) Require(card.names[i++] == name, "Glass card field name changed");
}
void Identity(assembly::SourceId id) {
    Require(id && id <= UINT32_MAX, "Invalid glass source identity");
}
assembly::Material ReadMaterial(const output::Value& value) {
    Require(value.IsObject() && value.MemberCount() == 12, "Unexpected glass material fields");
    TextIs(value,"material_law","layered_law44");
    TextIs(value,"hardening_model","law44_linear");
    assembly::Material material;
    ReadMaterialTuple(value, material);
    Identity(material.id);
    material.hardening = assembly::MaterialHardening::LinearLaw44;
    material.supplied_sigy_pa = Real(value,"supplied_sigy_pa");
    material.supplied_etan_pa = Real(value,"supplied_etan_pa");
    material.source = Block(Member(value,"source"));
    material.cards = Cards(value,4);
    Require(material.source.keyword == "*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY" ||
            material.source.keyword == "*MAT_123", "Glass declaration is not original MAT123");
    CheckGlassMaterialCards(material.cards);
    Names(material.cards[0], {"mid","ro","e","pr","sigy","etan","fail","tdel"});
    Names(material.cards[1], {"c","p","lcss","lcsr","vp","epsthin","epsmaj","numint"});
    Names(material.cards[2], {"eps1","eps2","eps3","eps4","eps5","eps6","eps7","eps8"});
    Names(material.cards[3], {"es1","es2","es3","es4","es5","es6","es7","es8"});
    CheckGlassCardOrder(material.source,material.cards);
    detail::CheckTypedCards(material.source,material.cards);
    const assembly::SourceUnits units{1000,.001,1};
    const auto& first = material.cards[0];
    Same(RequiredMaterialCard(first,0), double(material.id));
    Same(RequiredMaterialCard(first,1)*MaterialDensityScale(units), material.density_kg_m3);
    Same(RequiredMaterialCard(first,2)*MaterialStressScale(units), material.young_pa);
    Same(RequiredMaterialCard(first,3), material.poisson_ratio);
    Same(RequiredMaterialCard(first,4)*MaterialStressScale(units), *material.supplied_sigy_pa);
    Same(RequiredMaterialCard(first,5)*MaterialStressScale(units), *material.supplied_etan_pa);
    Same(RequiredMaterialCard(first,6), Real(value,"failure_strain"));
    Same(RequiredMaterialCard(material.cards[1],7), Real(value,"source_numint"));
    const double tangent = *material.supplied_etan_pa;
    const double hardening = tangent*material.young_pa/(material.young_pa-tangent);
    Require(material.density_kg_m3 > 0 && material.young_pa > tangent && tangent >= 0 &&
            *material.supplied_sigy_pa > 0 && std::isfinite(hardening) && hardening >= 0 &&
            (tangent == 0 || hardening > 0), "Glass coefficient overflow or underflow");
    // Generic rate/curve fields remain unused canonical values. Only the named
    // NativeGlassMaterial adapter resolves the separately authenticated policy.
    return material;
}
assembly::Section ReadSection(const output::Value& value) {
    Require(value.IsObject() && value.MemberCount() == 8, "Unexpected glass section fields");
    assembly::Section section;
    section.id = Unsigned(value,"section_id",UINT32_MAX);
    Identity(section.id);
    section.source_elform = Unsigned(value,"source_elform",2);
    section.through_thickness_points = Unsigned(value,"through_thickness_points",3);
    section.source = Block(Member(value,"source"));
    section.cards = Cards(value,2);
    Require(section.source.keyword == "*SECTION_SHELL", "Unsupported glass section keyword");
    CheckGlassSectionCards(section.cards);
    Names(section.cards[0], {"secid","elform","shrf","nip","propt","qr_irid","icomp","setyp"});
    Names(section.cards[1], {"t1","t2","t3","t4","nloc","marea","idof","edgset"});
    CheckGlassCardOrder(section.source,section.cards);
    detail::CheckTypedCards(section.source,section.cards);
    Same(RequiredMaterialCard(section.cards[0],0), double(section.id));
    Same(RequiredMaterialCard(section.cards[0],1), double(section.source_elform));
    Same(RequiredMaterialCard(section.cards[0],3), double(section.through_thickness_points));
    const auto& thickness = Array(value,"thickness_m",4,4);
    for (unsigned i = 0; i < 4; ++i) {
        section.thickness_m[i] = Real(thickness[i]);
        Same(RequiredMaterialCard(section.cards[1],i)*.001, section.thickness_m[i]);
        Require(section.thickness_m[i] > 0, "Glass thickness conversion underflow");
    }
    return section;
}
assembly::Part ReadPart(const output::Value& value) {
    Require(value.IsObject() && value.MemberCount() == 6, "Unexpected glass part fields");
    assembly::Part part;
    part.id = Unsigned(value,"part_id",UINT32_MAX);
    part.material_id = Unsigned(value,"material_id",UINT32_MAX);
    part.section_id = Unsigned(value,"section_id",UINT32_MAX);
    Identity(part.id);
    Identity(part.material_id);
    Identity(part.section_id);
    part.title = Text(value,"title");
    part.source = Block(Member(value,"source"));
    part.cards = Cards(value,1);
    Require(part.source.keyword == "*PART" && part.cards[0].blank_mask == 248,
            "Unsupported glass part options");
    Names(part.cards[0], {"pid","secid","mid","eosid","hgid","grav","adpopt","tmid"});
    detail::CheckTypedCards(part.source,part.cards);
    Same(RequiredMaterialCard(part.cards[0],0), double(part.id));
    Same(RequiredMaterialCard(part.cards[0],1), double(part.section_id));
    Same(RequiredMaterialCard(part.cards[0],2), double(part.material_id));
    return part;
}
}
GlassDeclaration ReadGlassDeclaration(const output::Value& value) {
    Require(value.IsObject() && value.MemberCount() == 3, "Unexpected glass declaration fields");
    GlassDeclaration next;
    next.part = ReadPart(Member(value,"part"));
    next.material = ReadMaterial(Member(value,"material"));
    const auto& section = Member(value,"section");
    next.section = ReadSection(section);
    Require(next.part.material_id == next.material.id && next.part.section_id == next.section.id,
            "Glass part material/section association changed");
    next.failure_strain = Real(Member(value,"material"),"failure_strain");
    next.source_nloc = OptionalReal(section,"source_nloc");
    const auto original = next.section.cards[1].values[4];
    Require(bool(original) == bool(next.source_nloc), "Original NLOC blank changed");
    if (original) Same(*original,*next.source_nloc);
    const double nloc = next.source_nloc.value_or(0);
    const char* name = "centered";
    if (nloc == -1) {
        next.placement = tl::fea::ShellReferencePlacement::BottomReferencePlane;
        name = "bottom_reference_plane";
    } else if (nloc == 1) {
        next.placement = tl::fea::ShellReferencePlacement::TopReferencePlane;
        name = "top_reference_plane";
    }
    TextIs(section,"placement",name);
    return next;
}
} // namespace crash::modelio::vehicle::resolution
