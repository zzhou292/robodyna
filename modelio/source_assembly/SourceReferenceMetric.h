#pragma once
#include "SourceAssemblyData.h"
#include "output/ArtifactIO.h"
#include "lib_src/elements/qeph/QephData.h"
#include <cmath>
namespace crash::modelio::assembly {
enum class QephMetricProfile { LegacyOneMetre, AuthenticatedSourceLength };
// Numerical source choice only, never owner/material equivalence authority.
// Factories resolve this from their already authenticated SourceUnits; callers
// cannot supply an unrelated free scalar through the factory profile API.
class QephReferenceMetric {
  public:
    static QephReferenceMetric Resolve(QephMetricProfile profile,const SourceUnits& units) {
        if(profile==QephMetricProfile::LegacyOneMetre)return {profile,1.};
        output::Require(profile==QephMetricProfile::AuthenticatedSourceLength,"Unknown QEPH source metric profile");
        output::Require(std::isfinite(units.mass_to_kg)&&units.mass_to_kg>0&&
            std::isfinite(units.length_to_m)&&units.length_to_m>0&&std::isfinite(units.time_to_s)&&units.time_to_s>0,
            "Source-working QEPH metric requires complete finite positive source units");
        return {profile,units.length_to_m};
    }
    QephMetricProfile profile() const noexcept{return profile_;}
    double working_length_m() const noexcept{return length_;}
  private:
    QephReferenceMetric(QephMetricProfile p,double length):profile_(p),length_(length){}
    QephMetricProfile profile_;double length_;
};
// Only the actual QEPH dispatch calls this. In particular, QBAT's internal
// quadrilateral is not QEPH execution and retains its legacy1m descriptor.
inline tl::fea::qeph::ReferenceInput WithQephMetric(tl::fea::qeph::ReferenceInput input,
    const QephReferenceMetric& metric) {
    input.projection_working_length_m=metric.working_length_m();return input;
}
}
