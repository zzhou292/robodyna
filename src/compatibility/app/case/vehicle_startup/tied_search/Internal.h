#pragma once
#include "../TiedSearchAssessment.h"
namespace crash::cases::vehicle_startup::tied_assessment_detail {
struct Inputs {
    std::vector<native_search::Vec3> positions;
    std::vector<native_search::SearchMasterInput> masters;
    native_search::SearchDriverInput View(const tied::SearchGeometryData&) const;
};
TiedAssessmentForecast Preflight(const tied::source::CanonicalData&, const tied::Data&,
    const tied::PackingData&, const tied::SearchGeometryData&, TiedAssessmentLimits);
std::size_t CanonicalPayload(const tied::source::CanonicalData&, std::size_t cap);
Inputs Pack(const tied::Data&, const tied::PackingData&, const tied::SearchGeometryData&);
std::size_t SelectedDeclarationRow(const tied::SearchGeometryData&,
    const native_search::SearchDriverResult&, std::size_t);
void CheckResult(const tied::Data&, const tied::SearchGeometryData&,
                 const native_search::SearchDriverResult&);
inline void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    output::Require(width && bytes <= cap && count <= (cap-bytes)/width,
                    "Tied assessment host byte cap exceeded");
    bytes += count*width;
}
} // namespace crash::cases::vehicle_startup::tied_assessment_detail
