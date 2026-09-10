#pragma once
#include "JsonReader.h"
namespace crash::modelio::assembly::reader {
inline void ReadMaterialTuple(const Value& value, Material& m) {
    m.id=Unsigned(value,"material_id",UINT32_MAX);
    m.density_kg_m3=Real(value,"density_kg_m3");m.young_pa=Real(value,"young_pa");
    m.poisson_ratio=Real(value,"poisson_ratio");
}
inline double RequiredMaterialCard(const DeclarationCard& card,std::size_t field) {
    Require(field<card.values.size()&&card.values[field],"Required material card field is blank");
    return *card.values[field];
}
inline double MaterialStressScale(const SourceUnits& u) {
    return u.mass_to_kg/(u.length_to_m*u.time_to_s*u.time_to_s);
}
inline double MaterialDensityScale(const SourceUnits& u) {
    return u.mass_to_kg/(u.length_to_m*u.length_to_m*u.length_to_m);
}
} // namespace crash::modelio::assembly::reader
