#include "MidlayerDeclarations.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include <cmath>

namespace crash::modelio::vehicle::resolution {
namespace {
using output::Require;
using assembly::reader::RequiredMaterialCard;
void Shape(const std::vector<assembly::DeclarationCard>& cards,
           std::initializer_list<unsigned> masks) {
    Require(cards.size() == masks.size(), "Midlayer card count changed");
    std::size_t i = 0;
    for (auto mask : masks) {
        Require(cards[i].values.size() == 8 && cards[i].blank_mask == mask,
                "Unsupported literal midlayer fields");
        ++i;
    }
}
double Scale(double value, double factor) {
    const double result = value * factor;
    Require(std::isfinite(result) && (value == 0 || result != 0),
            "Midlayer SI conversion overflow or underflow");
    return result;
}
}
MidlayerDeclaration ReadMidlayer(const PartDisposition& part, const assembly::SourceUnits& units) {
    Require(part.status == Disposition::Unresolved && part.part_id && part.part_id <= UINT32_MAX &&
            part.material_id && part.material_id <= UINT32_MAX &&
            part.section_id && part.section_id <= UINT32_MAX &&
            units.mass_to_kg == 1000 && units.length_to_m == .001 && units.time_to_s == 1,
            "Unsupported midlayer source identity or units");
    const auto& blocks = part.unresolved_sources;
    Require(blocks[0].keyword == "*PART" && blocks[1].keyword == "*SECTION_SHELL" &&
            (blocks[2].keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" || blocks[2].keyword == "*MAT_024"),
            "Unsupported midlayer source keywords");
    for (const auto& block : blocks) {
        Require(block.raw_text.size() <= 64*1024 && block.last_line >= block.first_line &&
                block.last_line-block.first_line < 1024, "Midlayer source card extent exceeds cap");
    }
    MidlayerDeclaration next;
    next.part_cards = detail::ReadSourceCards(blocks[0],1,1);
    next.section.cards = detail::ReadSourceCards(blocks[1],0,2);
    next.material.cards = detail::ReadSourceCards(blocks[2]);
    Shape(next.part_cards,{248});
    Shape(next.section.cards,{244,240});
    Shape(next.material.cards,{128,243,255,255});
    next.part_cards[0].names={"pid","secid","mid","eosid","hgid","grav","adpopt","tmid"};
    next.section.cards[0].names={"secid","elform","shrf","nip","propt","qr_irid","icomp","setyp"};
    next.section.cards[1].names={"t1","t2","t3","t4","nloc","marea","idof","edgset"};
    next.material.cards[0].names={"mid","ro","e","pr","sigy","etan","fail","tdel"};
    next.material.cards[1].names={"c","p","lcss","lcsr","vp","reserved6","reserved7","reserved8"};
    for (unsigned i=1;i<=8;++i) {
        next.material.cards[2].names.push_back("eps"+std::to_string(i));
        next.material.cards[3].names.push_back("es"+std::to_string(i));
    }
    for (unsigned i = 0; i < 3; ++i) {
        const auto& cards = i == 0 ? next.part_cards :
                            (i == 1 ? next.section.cards : next.material.cards);
        detail::CheckTypedCards(blocks[i],cards);
    }
    const auto at = RequiredMaterialCard;
    Require(at(next.part_cards[0],0) == double(part.part_id) &&
            at(next.part_cards[0],1) == double(part.section_id) &&
            at(next.part_cards[0],2) == double(part.material_id), "Midlayer PART association changed");
    auto& section = next.section;
    const auto& s = section.cards[0];
    Require(at(s,0) == double(part.section_id) && at(s,1) == 9 && at(s,3) == 1,
            "Midlayer requires literal ELFORM9 and NIP1");
    section.id = part.section_id;
    section.source_elform = 9;
    section.through_thickness_points = 1;
    section.source = blocks[1];
    for (unsigned i = 0; i < 4; ++i) {
        const double value = at(section.cards[1],i);
        Require(value > 0 && value == at(section.cards[1],0), "Midlayer thickness must be uniform positive");
        section.thickness_m[i] = Scale(value,units.length_to_m);
    }
    auto& material = next.material;
    const auto& m = material.cards[0];
    Require(at(m,0) == double(part.material_id) && at(m,1) > 0 && at(m,2) > 0 &&
            at(m,3) >= 0 && at(m,3) < .5 && at(m,4) > 0 && at(m,5) >= 0 &&
            at(m,5) < at(m,2) && at(m,6) > 0 &&
            output::Bits(at(material.cards[1],2)) == output::Bits(0.0) &&
            output::Bits(at(material.cards[1],3)) == output::Bits(0.0),
            "Midlayer requires analytic MAT024 with positive FAIL and explicit LCSS0/LCSR0");
    const double stress = assembly::reader::MaterialStressScale(units);
    material.id = part.material_id;
    material.density_kg_m3 = Scale(at(m,1),assembly::reader::MaterialDensityScale(units));
    material.young_pa = Scale(at(m,2),stress);
    material.poisson_ratio = at(m,3);
    material.supplied_sigy_pa = Scale(at(m,4),stress);
    material.supplied_etan_pa = Scale(at(m,5),stress);
    material.hardening = assembly::MaterialHardening::LinearLaw44;
    material.source = blocks[2];
    next.failure_strain = at(m,6);
    const auto native = assembly::detail::NativeMaterial(material,assembly::detail::NativeLaw44Rate::FilteredZeroC);
    tl::material::TabulatedShellPlasticityParameters parameters;
    Require(tl::material::PrepareLinearLaw44ShellPlasticity(native.young_pa,native.poisson_ratio,
            native.density_kg_m3,native.linear,native.rate,parameters) ==
            tl::material::TabulatedShellPlasticityStatus::Ok, "Invalid native midlayer material");
    return next;
}
} // namespace crash::modelio::vehicle::resolution
