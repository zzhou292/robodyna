#include "Internal.h"
#include "AirbagSourceReceipt.h"
#include "lib_src/materials/law44/solid/Prepare.h"

namespace crash::modelio::solid_source::detail {
void ReadAirbagMaterial(Part& part, const tied_shell::SourceEvidence& source, const Data& data) {
    Require(SelectedAirbag(part.id, data.policy) &&
            source.block.keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" && source.cards.size() == 4 &&
            part.curve_source == SIZE_MAX, "Original airbag requires analytic MAT024 without a curve");
    const auto& first = source.cards[0].second;
    const auto& controls = source.cards[1].second;
    Require(Required(first, 1) == 1.9500e-9 && Required(first, 2) == 1000 &&
            Required(first, 3) == .3 && Required(first, 4) == 20 && Required(first, 5) == 10,
            "Original airbag density/elastic/analytic-hardening declaration changed");
    Blank(first, 6, 8); // FAIL/TDEL remain blank; EPSGM and EPMAX are native distinct defaults.
    Require(Required(controls, 0) == 8000 && Required(controls, 1) == 8 &&
            Required(controls, 2) == 0 && !vehicle::detail::SourceScalar(controls, 3) &&
            Required(controls, 4) == 0, "Original airbag filtered-rate analytic branch changed");
    Blank(controls, 5, 8);
    Blank(source.cards[2].second, 0, 8);
    Blank(source.cards[3].second, 0, 8);
    std::size_t hourglass = SIZE_MAX;
    for (std::size_t i = 0; i < data.sources.size(); ++i) {
        const auto& hg = data.sources[i];
        if (hg.block.keyword != "*CONTROL_HOURGLASS") continue;
        Require(hourglass == SIZE_MAX && hg.block.filename == "combine.key" &&
                hg.block.first_line == 142 && hg.block.last_line == 148 &&
                hg.block.sha256 == AirbagHourglassHash &&
                output::Sha256(hg.block.raw_text) == AirbagHourglassHash && hg.cards.size() == 1 &&
                hg.cards[0].first == 145 && Required(hg.cards[0].second, 0) == 4 &&
                Required(hg.cards[0].second, 1) == .02,
                "Original airbag global hourglass proof changed");
        Blank(hg.cards[0].second, 2, 8);
        hourglass = i;
    }
    Require(hourglass != SIZE_MAX, "Missing original airbag global hourglass proof");

    namespace point = tl::material::law44::solid;
    point::Material material;
    material.young_pa = Required(first, 2) * 1e6;
    material.poisson_ratio = Required(first, 3);
    material.density_kg_m3 = Required(first, 1) * 1e12;
    material.rate_c_per_s = Required(controls, 0);
    material.rate_p = Required(controls, 1);
    material.cutoff_hz = 10000;
    material.native_units = point::WorkingUnits::TonneMillimetreSecond;
    point::Parameters prepared;
    Require(point::PrepareMat024Analytic(material, Required(first, 2), Required(first, 4),
                Required(first, 5), prepared) == point::Status::Ok,
            "Original airbag analytic LAW44 preparation failed");
    part.material_law = MaterialLaw::Law44;
    part.density_kg_m3 = material.density_kg_m3;
    part.law44 = prepared;
    part.hourglass_source = hourglass;
    part.original_ihq = 4;
    part.original_qh = .02;
    part.converter_isolid = 5;
    part.selected_isolid = 18;
}
} // namespace crash::modelio::solid_source::detail
