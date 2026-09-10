#include "SourcePartPlasticPilot.h"
#include "case/source_part_elastic/SourcePartElasticPilot.h"
#include <cmath>
#include <stdexcept>

namespace crash::cases::source_part_plastic {
source_part_elastic::Config PlasticWallConfig(const SourcePartMaterial& material,unsigned refinement,double speed) {
    if(!material.prepared()||!std::isfinite(speed)||speed<1||speed>16)
        throw std::invalid_argument("Plastic impact requires the authenticated material and a speed in [1,16] m/s");
    auto c=source_part_elastic::MeshWallConfig(refinement);
    c.material_model=source_part_elastic::MaterialModel::SourceCowperSymonds;
    c.material=material;
    const auto& m=material.declaration();
    c.rate={true,m.source_rate_coefficient_per_s,m.source_rate_exponent,m.resolved_filter_cutoff_per_s};
    c.initial_velocity={speed,0,0};
    // These are declared experimental geometry envelopes, not changed moduli.
    // Curve-domain, contact-step, native-step, work and 5% total-energy checks
    // remain independently active in the owner/contributors.
    c.maximum_displacement=.20;
    c.maximum_rotation=1.5;
    c.maximum_strain=.15;
    c.maximum_thickness_curvature=.25;
    c.minimum_area_ratio=.70; c.maximum_area_ratio=1.30;
    c.minimum_thickness_ratio=.70; c.maximum_thickness_ratio=1.30;
    c.configuration_id=0x5350504c41533031ULL;
    c.qualification_id=0x5350504c41514c31ULL;
    if(!source_part_elastic::ValidConfig(c)) throw std::invalid_argument("Invalid source plastic impact configuration");
    return c;
}
source_part_wall::SourcePartWallSettings PlasticWallSettings(const source_part_elastic::Config& c) {
    auto settings=source_part_elastic::MeshWallSettings(c);
    settings.leading_gap=.020; // 20 mm approach, visible at physical scale.
    settings.motion_margin=c.maximum_displacement;
    settings.wall_binding_id=0x5350504c57414c31ULL;
    return settings;
}
}
