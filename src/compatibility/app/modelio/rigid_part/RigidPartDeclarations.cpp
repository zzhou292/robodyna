#include "RigidPartDeclarations.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include <cmath>
namespace crash::modelio::vehicle::rigid_part {
namespace {
using output::Require;
using assembly::reader::RequiredMaterialCard;
void OptionalZero(const assembly::DeclarationCard& card, unsigned first) {
    for (unsigned i=first;i<8;++i)
        Require(!card.values[i] || *card.values[i]==0, "Unsupported rigid source option");
}
double Scaled(double value,double factor) {
    const double result=value*factor;
    Require(std::isfinite(result)&&result>0,"Invalid rigid SI coefficient");
    return result;
}
}
Declaration ReadDeclaration(const PartDisposition& p,const assembly::SourceUnits& units) {
    Require(p.status==Disposition::Unresolved && p.part_id && p.material_id && p.section_id &&
            p.part_id<=UINT32_MAX && p.material_id<=UINT32_MAX && p.section_id<=UINT32_MAX &&
            units.mass_to_kg==1000 && units.length_to_m==.001 && units.time_to_s==1,
            "Invalid original rigid identity or units");
    const auto& b=p.unresolved_sources;
    Require(b[0].keyword=="*PART" && b[1].keyword=="*SECTION_SHELL" && b[2].keyword=="*MAT_RIGID",
            "Unsupported rigid source keyword");
    for (const auto& block:b)
        Require(block.raw_text.size()<=64*1024 && block.last_line>=block.first_line &&
                block.last_line-block.first_line<1024,"Rigid source block exceeds cap");
    Declaration out;
    out.part_cards=detail::ReadSourceCards(b[0],1,1);
    out.section.cards=detail::ReadSourceCards(b[1],0,2);
    out.material.cards=detail::ReadSourceCards(b[2],0,3);
    Require(out.part_cards.size()==1 && out.section.cards.size()==2 && out.material.cards.size()==3,
            "Rigid source card count changed");
    const auto at=RequiredMaterialCard;
    Require(at(out.part_cards[0],0)==double(p.part_id) &&
            at(out.part_cards[0],1)==double(p.section_id) && at(out.part_cards[0],2)==double(p.material_id),
            "Rigid PART association changed");
    OptionalZero(out.part_cards[0],3);
    auto& s=out.section;
    Require(at(s.cards[0],0)==double(p.section_id) && at(s.cards[0],1)==2 && at(s.cards[0],3)==3,
            "Rigid shell requires original ELFORM2/NIP3");
    Require(!s.cards[0].values[2],"Rigid shear factor must retain its original blank default");
    OptionalZero(s.cards[0],4);
    OptionalZero(s.cards[1],4);
    s.id=p.section_id; s.source_elform=2; s.through_thickness_points=3; s.source=b[1];
    for (unsigned i=0;i<4;++i) {
        Require(at(s.cards[1],i)==at(s.cards[1],0),"Rigid thickness is not uniform");
        s.thickness_m[i]=Scaled(at(s.cards[1],i),units.length_to_m);
    }
    auto& m=out.material;
    Require(at(m.cards[0],0)==double(p.material_id) && at(m.cards[0],3)==.3,
            "Original rigid material identity/Poisson ratio changed");
    OptionalZero(m.cards[0],4);
    // CMO/CON1/CON2 and supplied axes/velocity are not admitted by this profile.
    Require(m.cards[1].blank_mask==255 && m.cards[2].blank_mask==255,
            "Rigid body constraint/axes options must remain blank");
    m.id=p.material_id; m.law=assembly::MaterialLaw::LayeredLaw1;
    m.density_kg_m3=Scaled(at(m.cards[0],1),assembly::reader::MaterialDensityScale(units));
    m.young_pa=Scaled(at(m.cards[0],2),assembly::reader::MaterialStressScale(units));
    m.poisson_ratio=at(m.cards[0],3); m.source=b[2];
    out.part_cards[0].names={"pid","secid","mid","eosid","hgid","grav","adpopt","tmid"};
    s.cards[0].names={"secid","elform","shrf","nip","propt","qr_irid","icomp","setyp"};
    s.cards[1].names={"t1","t2","t3","t4","nloc","marea","idof","edgset"};
    m.cards[0].names={"mid","ro","e","pr","n","couple","m","alias"};
    m.cards[1].names={"cmo","con1","con2","reserved4","reserved5","reserved6","reserved7","reserved8"};
    m.cards[2].names={"lco_or_a1","a2","a3","v1","v2","v3","reserved7","reserved8"};
    detail::CheckTypedCards(b[0],out.part_cards);
    detail::CheckTypedCards(b[1],s.cards);
    detail::CheckTypedCards(b[2],m.cards);
    (void)assembly::detail::NativeMaterial(m);
    return out;
}
}
