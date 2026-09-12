#include "Internal.h"
namespace crash::modelio::beam18::detail {
void ReadPart(Part& part,Data& data) {
    const auto& original=data.sources.at(part.sources[0]);
    const auto& section=data.sources.at(part.sources[1]);
    const auto& material=data.sources.at(part.sources[2]);
    Require(Selected(part.id) && original.block.keyword=="*PART" && original.cards.size()==2 &&
        section.block.keyword=="*SECTION_BEAM" && section.cards.size()==2 &&
        material.block.keyword=="*MAT_PIECEWISE_LINEAR_PLASTICITY" && material.cards.size()==4,
        "Original beam18 declaration family or card count changed");
    const auto& p=original.cards[1].second;
    const auto& s=section.cards[0].second;
    const auto& geometry=section.cards[1].second;
    const auto& first=material.cards[0].second;
    const auto& controls=material.cards[1].second;
    Require(part.id==part.section_id && part.id==part.material_id &&
        tied_shell::detail::CardId(p,0)==part.id && tied_shell::detail::CardId(p,1)==part.section_id &&
        tied_shell::detail::CardId(p,2)==part.material_id && tied_shell::detail::CardId(s,0)==part.section_id &&
        tied_shell::detail::CardId(first,0)==part.material_id,"Original beam18 source identity changed");
    RequireBlankFields(p,3,8);
    Require(RequiredScalar(s,1)==1 && RequiredScalar(s,3)==0 && RequiredScalar(s,4)==1,
        "Beam18 requires original ELFORM1 QR0 CST1");
    if(part.id==2000948) Require(RequiredScalar(s,2)==1,"Original headrest SHRF changed");
    else Require(!SourceScalar(s,2),"Original blank beam SHRF changed");
    RequireBlankFields(s,5,8);
    const double diameter=part.id<2000900 ? 9. : 11.8;
    Require(RequiredScalar(geometry,0)==diameter && RequiredScalar(geometry,1)==diameter,
        "Original circular beam diameters changed");
    RequireBlankFields(geometry,2,8);
    part.radius_mm=std::max(RequiredScalar(geometry,0)/2.,RequiredScalar(geometry,1)/2.);
    Require(RequiredScalar(first,1)==7.89e-9 && RequiredScalar(first,2)==200000 &&
        RequiredScalar(first,3)==.3 && RequiredScalar(first,4)==270,"Original beam18 material values changed");
    RequireBlankFields(first,5,8);
    Require(RequiredScalar(controls,0)==8000 && RequiredScalar(controls,1)==8 &&
        tied_shell::detail::CardId(controls,2)==2100270 && !SourceScalar(controls,3) && RequiredScalar(controls,4)==0,
        "Original beam18 table/rate branch changed");
    RequireBlankFields(controls,5,8);
    RequireBlankFields(material.cards[2].second,0,8);RequireBlankFields(material.cards[3].second,0,8);
    part.density_working=RequiredScalar(first,1);part.young_working=RequiredScalar(first,2);
    auto& m=part.material.material;
    m.young_pa=part.young_working*1e6;m.poisson_ratio=RequiredScalar(first,3);
    m.density_kg_m3=part.density_working*1e12;
    m.rate_c_per_s=RequiredScalar(controls,0);m.rate_p=RequiredScalar(controls,1);
    m.cutoff_hz=10000;m.native_units=tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond;
    const auto& curve=data.sources.at(part.curve_source);
    ReadSourceCurve(curve.block,curve.cards,2100270,46,1e6,data.plastic_strain,data.yield_stress_pa);
}
} // namespace crash::modelio::beam18::detail
