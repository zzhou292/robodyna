#include "ResolvedSpotweldProperty.h"

namespace crash::modelio::assembly {
tl::fea::type25::Property ResolvedSpotweldProperty() {
    // Pinned mm_s_Mg row: unitsystemdefaults.cxx88-107; Ileng0 dimensions:
    // prop_p25_spr_axi.cfg328-329,355 and corresponding rotational channels.
    const double mass_unit = SpotweldSourceUnits.mass_to_kg;
    const double inertia_unit = mass_unit * SpotweldSourceUnits.length_to_m * SpotweldSourceUnits.length_to_m;
    const double force_unit = mass_unit * SpotweldSourceUnits.length_to_m /
        (SpotweldSourceUnits.time_to_s * SpotweldSourceUnits.time_to_s);
    const double moment_unit = force_unit * SpotweldSourceUnits.length_to_m;
    tl::fea::type25::Property p;
    p.mass_kg = .001e-3 * mass_unit;
    p.isotropic_inertia_kg_m2 = .01e-3 * inertia_unit;
    p.stiffness[0] = p.stiffness[1] = 100.e3 * mass_unit /
        (SpotweldSourceUnits.time_to_s * SpotweldSourceUnits.time_to_s);
    p.stiffness[2] = p.stiffness[3] = 1000.e3 * inertia_unit /
        (SpotweldSourceUnits.time_to_s * SpotweldSourceUnits.time_to_s);
    // Blank SN/SS/N/M resolve to0 at direct SDI import; HM_READ_PROP25 replaces
    // zero rupture limits with +/-EP30*channel units and zero alpha/beta with
    //1/2. Missing damping is0. These are explicit converter regularizers.
    for (unsigned c = 0; c < 4; ++c) {
        p.failure_positive[c] = 1.e30 * (c < 2 ? force_unit : moment_unit);
        p.failure_negative[c] = -p.failure_positive[c];
        p.failure_weight[c] = 1.;
        p.failure_exponent[c] = 2.;
    }
    return p;
}
}
