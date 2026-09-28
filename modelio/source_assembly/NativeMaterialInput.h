#pragma once
#include "SourceAssemblyMaterialInput.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::assembly::detail {
enum class NativeLaw44Rate { SuppliedPositive, FilteredZeroC };
inline tl::fea::ShellPlasticityMaterialInput NativeMaterial(const Material& material,
    NativeLaw44Rate rate_policy = NativeLaw44Rate::SuppliedPositive) {
    output::Require(rate_policy == NativeLaw44Rate::SuppliedPositive ||
                    rate_policy == NativeLaw44Rate::FilteredZeroC, "Invalid native rate policy");
    // Unused plastic controls have the catalog's canonical default values;
    // they are not elastic source declarations or invented material history.
    tl::fea::ShellPlasticityMaterialInput native{material.id, material.curve_id,
        material.young_pa, material.poisson_ratio, material.density_kg_m3, {}};
    if(material.law==MaterialLaw::LayeredLaw1) {
        output::Require(rate_policy == NativeLaw44Rate::SuppliedPositive && !material.curve_id,
                        "Elastic source material cannot reference a plastic curve");
        native.law=tl::fea::ShellSectionLaw::LayeredLaw1Nip3;
        return native;
    }
    output::Require(material.law==MaterialLaw::LayeredLaw44&&material.source_rate_type==0,
        "Assembly direct-import policy requires source LAW44 VP=0");
    // Pinned direct import: absent Fcut becomes zero in CPP_GET_FLOATV_FLOATD;
    // HM_READ_MAT44 with ISMOOTH=1 resolves 10000/s, not a CFG re-read default.
    native.rate={true,material.rate_c_per_s,material.rate_p,10000.};
    if (rate_policy == NativeLaw44Rate::FilteredZeroC) {
        output::Require(material.hardening == MaterialHardening::LinearLaw44 && !material.curve_id,
                        "Filtered zero-C source policy requires analytic LAW44");
        native.rate={true,0,1,10000,tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
    }
    if (material.hardening == MaterialHardening::TabulatedLaw44) {
        output::Require(material.curve_id != 0, "Tabulated native LAW44 requires its source curve");
        // Native SIGEPS44C calls VINTER independently of the failure card.
        // The final source knot selects a segment; it is not a strain limit.
        // Standalone TL material preparation keeps its explicit strict default.
        native.continuation = tl::material::ShellPlasticityCurveContinuation::NativeLastSegment;
    } else {
        output::Require(material.hardening == MaterialHardening::LinearLaw44,
                        "Unsupported native LAW44 hardening declaration");
        output::Require(material.supplied_sigy_pa&&material.supplied_etan_pa&&!material.curve_id,
            "Analytic source material requires explicit SIGY/ETAN and no curve");
        native.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
        native.linear={*material.supplied_sigy_pa,*material.supplied_etan_pa};
    }
    return native;
}
} // namespace crash::modelio::assembly::detail
