#pragma once
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <cstdint>
namespace crash::modelio::native_scene {
// Source H is the native plastic modulus in SI, not an input tangent modulus.
// ETAN uses the exact association H/(1+H/E). No scene-specific nextafter
// adjustment or new mechanics initializer occurs.
struct LinearHardeningBridge {
    double source_h_pa=0,derived_etan_pa=0,prepared_h_pa=0;
    std::uint64_t prepared_h_ulp_difference=0;
    bool exact_source_h_identity=false;
    tl::material::TabulatedShellPlasticityParameters parameters;
};
// Explicit supported source range: finite E>0, 0<=H<=E, existing qualified
// elastic/rate controls. A derived value must reconstruct H within2ULP through
// the ACTUAL unchanged TL preparation. Throws without publishing on failure.
LinearHardeningBridge PrepareNativeHardening(double young_pa,double poisson,double density_kg_m3,
    double source_yield_pa,double source_h_pa,tl::material::TabulatedShellPlasticityRate);
} // namespace crash::modelio::native_scene
