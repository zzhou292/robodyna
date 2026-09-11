#include "GlassDeclarations.h"

namespace crash::modelio::vehicle::resolution {
using namespace assembly::reader;
void CheckGlassPolicy(const output::Value& value) {
    TextIs(value, "schema", "robo-dyna.vehicle-glass-declarations.v1");
    const auto& policy = Member(value, "policy");
    TextIs(policy, "name", "original_mat123_numint1_native_tab1_placement_v1");
    TextIs(policy, "revision", "a62b27e6baa555d222a580d6218867d0be4d70b5");
    TextIs(policy, "material_source", "convertmats.cxx:6170-6220,6390-6424");
    TextIs(policy, "failure", "Tab1AnyPoint");
    Require(Unsigned(policy,"source_NUMINT") == 1 && Unsigned(policy,"converter_IFAIL_SH") == 2 &&
            Unsigned(policy,"resolved_IFAIL_SH") == 1 && Unsigned(policy,"NIP") == 3,
            "Original NUMINT and native any-point policy changed");
    Same(Real(policy,"resolved_PTHKF"), 1e-6);
    TextIs(policy,"rate_policy","FilteredZeroC");
    Same(Real(policy,"resolved_C"), 0);
    Same(Real(policy,"resolved_P"), 1);
    Same(Real(policy,"resolved_VP"), 2);
    Same(Real(policy,"rate_filter_hz"), 10000);
    Same(Real(policy,"source_time_to_s"), 1);
    Flag(policy,"point_eps_max_cleared",true);
    Flag(policy,"FLD",false);
    TextIs(policy,"placement_policy","original_nloc_native_type1_reference_plane_v1");
    const auto& mapping = Array(policy,"nloc_to_ipos",3,3);
    const double expected[3][2]{{-1,4},{0,0},{1,3}};
    for (unsigned i = 0; i < 3; ++i) {
        Require(mapping[i].IsArray() && mapping[i].Size() == 2, "Invalid NLOC/IPOS mapping");
        for (unsigned j = 0; j < 2; ++j) Same(Real(mapping[i][j]), expected[i][j]);
    }
    Flag(policy,"contact_projection",false);
    Flag(policy,"simulation_ready",false);
}
tl::fea::ShellPlasticityMaterialInput NativeGlassMaterial(const assembly::Material& material) {
    tl::fea::ShellPlasticityMaterialInput native{material.id, 0, material.young_pa,
        material.poisson_ratio, material.density_kg_m3, {}};
    native.hardening = tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    native.linear = {*material.supplied_sigy_pa, *material.supplied_etan_pa};
    native.rate = {true, 0, 1, 10000, tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
    return native;
}
} // namespace crash::modelio::vehicle::resolution
