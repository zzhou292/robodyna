#include "Internal.h"
#include "lib_src/materials/law36/Prepare.h"
#include "lib_src/materials/law42/Prepare.h"

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
