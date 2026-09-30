#pragma once

namespace crash::reference {
// Two rectangular Q4s, six shared physical nodes, one centered isotropic layer.
// These SI parameters describe a prescribed reference fixture, not a material
// deck or a dynamics admission. The original B2/D constructors retain defaults.
struct ElasticCouponParameters {
    double length=.2,width=.1,thickness=.02;
    double young_modulus=1.2e6,poisson_ratio=.3,density=1000;
    double shear_factor=5.0/6.0,torque_factor=.01;
};
inline constexpr ElasticCouponParameters kElasticCouponDefaultParameters{};
inline constexpr bool IsDefaultElasticCouponParameters(const ElasticCouponParameters& p) noexcept {
    const auto& d=kElasticCouponDefaultParameters;
    return p.length==d.length&&p.width==d.width&&p.thickness==d.thickness&&
        p.young_modulus==d.young_modulus&&p.poisson_ratio==d.poisson_ratio&&p.density==d.density&&
        p.shear_factor==d.shear_factor&&p.torque_factor==d.torque_factor;
}
} // namespace crash::reference
