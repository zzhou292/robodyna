#include "Internal.h"
#include "lib_src/materials/law36/Prepare.h"
#include "lib_src/materials/law42/Prepare.h"
#include "lib_src/materials/law44/solid/Prepare.h"
#include "lib_src/materials/law90/Prepare.h"

namespace crash::modelio::solid_source::detail {
void PrepareMaterial(Part& part, Data& data) {
    if (part.id == AdhesivePart) {
        const tl::material::law36::Curve curve{data.plastic_strain.data(), data.yield_stress_pa.data(),
            static_cast<std::uint32_t>(data.plastic_strain.size())};
        tl::material::law36::Parameters material;
        const auto status = tl::material::law36::Prepare(part.law36.young_pa, part.law36.poisson_ratio,
            part.density_kg_m3, curve, material);
        Require(status == tl::material::law36::Status::Ok, "Native adhesive LAW36 material rejected");
        part.law36 = material;
    } else if (part.material_law == MaterialLaw::Law44) {
        const tl::material::law44::solid::Curve curve{data.rear_plastic_strain.data(),
            data.rear_yield_stress_pa.data(), static_cast<std::uint32_t>(data.rear_plastic_strain.size())};
        tl::material::law44::solid::Parameters material;
        const auto status = tl::material::law44::solid::Prepare(part.law44.material, curve, material);
        Require(status == tl::material::law44::solid::Status::Ok, "Native rear LAW44 material rejected");
        part.law44 = material;
    } else if (part.material_law == MaterialLaw::Law90) {
        const tl::material::law90::CurveView curve{data.foam_compression_strain.data(),
            data.foam_curve_ordinate.data(), static_cast<std::uint32_t>(data.foam_compression_strain.size())};
        const auto status = tl::material::law90::PrepareSI(part.law90_input, curve, part.law90);
        Require(status == tl::material::law90::Status::Ok && part.law90.reader().loading_flag == 1,
                "Original blank-HU radiator LAW90 material rejected");
    } else {
        // Pinned MAT007->LAW42: one Ogden term, alpha2, nu.463, no Prony.
        // Starter default tension cutoff is1e20 in the original MPa units.
        tl::material::law42::Parameters material;
        const auto status = tl::material::law42::Prepare(part.law42.mu_pa, .463,
            part.density_kg_m3, 1e20 * 1e6, material);
        Require(status == tl::material::law42::Status::Ok, "Native rubber LAW42 material rejected");
        part.law42 = material;
    }
}
} // namespace crash::modelio::solid_source::detail
