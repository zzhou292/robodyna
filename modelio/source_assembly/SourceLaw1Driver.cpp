#include "SourceLaw1Driver.h"
#include <cmath>

namespace crash::modelio::assembly {
namespace {
bool Supplied(const DeclarationCard& card, unsigned field, double value) noexcept {
    return field < card.values.size() && card.values[field] && *card.values[field] == value;
}
bool Blanks(const DeclarationCard& card, unsigned mask) noexcept {
    if (card.values.size() != 8 || card.blank_mask != mask) return false;
    for (unsigned field = 0; field < 8; ++field) {
        if (bool(card.values[field]) == bool(mask & (1u << field))) return false;
    }
    return true;
}
} // namespace

SourceLaw1Driver ResolveLaw1SourceDriver(const Material& material, const Section& section) noexcept {
    SourceLaw1Driver result;
    result.material_ = material.id;
    result.section_ = section.id;
    if (!material.id || !section.id) return result;
    if (material.source.keyword != "*MAT_ELASTIC") {
        if (material.law == MaterialLaw::LayeredLaw44 ||
            (material.law == MaterialLaw::LayeredLaw1 && material.source.keyword == "*MAT_RIGID")) {
            result.status_ = Law1DriverStatus::NotElastic;
        }
        return result;
    }
    if (material.law != MaterialLaw::LayeredLaw1 || material.curve_id || material.cards.size() != 1 ||
        section.source.keyword != "*SECTION_SHELL" || section.cards.size() != 2 ||
        (section.source_elform != 2 && section.source_elform != 16) || section.through_thickness_points != 3 ||
        !Blanks(material.cards[0], 240) || !Blanks(section.cards[0], 244) || !Blanks(section.cards[1], 240) ||
        !Supplied(material.cards[0], 0, double(material.id)) || !Supplied(section.cards[0], 0, double(section.id)) ||
        !Supplied(section.cards[0], 1, double(section.source_elform)) || !Supplied(section.cards[0], 3, 3)) {
        return result;
    }
    if (!std::isfinite(material.young_pa) || material.young_pa <= 0 ||
        !std::isfinite(material.density_kg_m3) || material.density_kg_m3 <= 0 ||
        !std::isfinite(material.poisson_ratio) || material.poisson_ratio < 0 || material.poisson_ratio >= .5) {
        return result;
    }
    for (unsigned slot = 0; slot < 4; ++slot) {
        if (!std::isfinite(section.thickness_m[slot]) || section.thickness_m[slot] <= 0 ||
            section.thickness_m[slot] != section.thickness_m[0]) return result;
    }
    // convertprops.cxx776–813: these exact ordinary source options select
    // TYPE1, Ishell24/Ish3n2, Ismstr2/ITHICK1/IPLAS1. Independently,
    // CGRTAILS and C3GRTAILS rewrite LAW1 NPN>1 to native NPT0.
    result.status_ = Law1DriverStatus::NativeA62Type1;
    result.elform_ = section.source_elform;
    return result;
}
} // namespace crash::modelio::assembly
