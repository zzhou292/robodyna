#pragma once
#include "lib_src/elements/qbat/QbatTypes.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::detail {
inline tl::fea::qbat::ReferenceInput OriginalMidlayerQbatInput(
    const tl::fea::qeph::ReferenceInput& quad, const tl::fea::ShellPlasticityMaterialInput& native) {
    output::Require(quad.placement == tl::fea::ShellReferencePlacement::Centered &&
        quad.young_modulus == native.young_pa && quad.poisson_ratio == native.poisson_ratio &&
        quad.density == native.density_kg_m3 &&
        native.hardening == tl::material::ShellPlasticityHardeningKind::LinearLaw44 &&
        native.rate.policy == tl::material::ShellPlasticityRatePolicy::FilteredZeroC,
        "Midlayer QBAT reference/material association changed");
    tl::material::TabulatedShellPlasticityParameters parameters;
    output::Require(tl::material::PrepareLinearLaw44ShellPlasticity(native.young_pa,native.poisson_ratio,
        native.density_kg_m3,native.linear,native.rate,parameters) ==
        tl::material::TabulatedShellPlasticityStatus::Ok, "Invalid virgin QBAT material");
    tl::fea::qbat::ReferenceInput input;
    input.quadrilateral = quad;
    // Original ELFORM9/NIP1 -> Ishell12/IHBE11. Values are explicit startup
    // policy, never a QEPH force selector or per-step material override.
    auto& options = input.options;
    options.ihbe = 11;
    options.irep = 0;
    options.ismstr = 2;
    options.nptr = options.npts = 2;
    options.nptt = options.layers = 1;
    options.idrill = options.npinch = 0;
    options.material_law = 44;
    options.property_type = 1;
    options.ithick = options.iplas = 1;
    options.offset_ratio = 0;
    options.inertia_denominator_override = 0;
    options.membrane_viscosity = 0;
    options.numerical_viscosity = 0;
    input.initial_a11_pa = parameters.a11;
    return input;
}
} // namespace crash::cases::vehicle_startup::detail
