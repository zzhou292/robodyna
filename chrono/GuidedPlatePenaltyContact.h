#pragma once

#include "GuidedPlateModel.h"
#include "collision/Q4RectangularIntegration.h"
#include <memory>

namespace crash::reference::penalty_detail {
namespace sc=tlfea::contact;
struct ContactSample {
    sc::Q4CertifiedIntegral potential,resultant;
    std::array<sc::Q4IntegralInterval,kCouponNodes> nodal_magnitude{};
    std::array<double,kCouponNodes> force_x{};
    double strip_potential=0,strip_resultant=0,strip_energy_allowance=0,strip_force_allowance=0;
};
// One bounded host workspace reused by both parents and all prescribed samples.
// No CUDA allocation and no persistent physical state.
struct ContactScratch {
    std::unique_ptr<sc::Q4RectangularCell[]> leaves{new sc::Q4RectangularCell[sc::MaxQ4IntegrationLeaves]};
    std::unique_ptr<std::uint32_t[]> heap{new std::uint32_t[sc::MaxQ4IntegrationLeaves]};
    sc::Q4RectangularScratch view() {
        return {leaves.get(),heap.get(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};
    }
};
ElasticCouponStatus Contact(const GuidedPlateModel&,const ElasticCouponConfiguration&,ContactScratch&,
                           ContactSample&,std::string&);
} // namespace crash::reference::penalty_detail
