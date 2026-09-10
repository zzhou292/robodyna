#pragma once
#include "SourceAssemblyMaterialInput.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::assembly::detail {
inline tl::fea::ShellPlasticityMaterialInput NativeMaterial(const Material& material) {
    // Unused plastic controls have the catalog's canonical default values;
    // they are not elastic source declarations or invented material history.
    tl::fea::ShellPlasticityMaterialInput native{material.id, material.curve_id,
        material.young_pa, material.poisson_ratio, material.density_kg_m3, {}};
    if(material.law==MaterialLaw::LayeredLaw1) {
        output::Require(!material.curve_id,"Elastic source material cannot reference a plastic curve");
        native.law=tl::fea::ShellSectionLaw::LayeredLaw1Nip3;
        return native;
    }
    output::Require(material.law==MaterialLaw::LayeredLaw44&&material.source_rate_type==0,
        "Assembly direct-import policy requires source LAW44 VP=0");
    // Pinned direct import: absent Fcut becomes zero in CPP_GET_FLOATV_FLOATD;
    // HM_READ_MAT44 with ISMOOTH=1 resolves 10000/s, not a CFG re-read default.
    native.rate={true,material.rate_c_per_s,material.rate_p,10000.};
    if(material.hardening==MaterialHardening::LinearLaw44) {
        output::Require(material.supplied_sigy_pa&&material.supplied_etan_pa&&!material.curve_id,
            "Analytic source material requires explicit SIGY/ETAN and no curve");
        native.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
        native.linear={*material.supplied_sigy_pa,*material.supplied_etan_pa};
    }
    return native;
}
} // namespace crash::modelio::assembly::detail
