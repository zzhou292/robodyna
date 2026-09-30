#include "ReferenceComparison.h"
#include "modelio/vehicle_sections/tests/RigidSourceSupport.h"
#include "modelio/source_assembly/tests/SectionTestSupport.h"
#include "modelio/source_assembly/SourceAssemblyShellInput.h"
#include "../ReferenceStorage.h"
namespace crash::cases::vehicle_startup::test {
TEST(VehicleReferenceMetricSource, CompleteOriginalMixedAssessmentPreservesRolesAndNonQephInputs) {
    const auto& resolution=modelio::vehicle::test::RigidResolution();const auto limits=ReferenceLimits::CompleteRigidOverlay();
    const auto legacy=VehicleShellReferences::Prepare(resolution,limits);
    const auto selected=VehicleShellReferences::Prepare(resolution,QephMetricProfile::AuthenticatedSourceLength,limits);
    EXPECT_EQ(legacy.qeph_metric().profile(),QephMetricProfile::LegacyOneMetre);
    EXPECT_EQ(selected.qeph_metric().profile(),QephMetricProfile::AuthenticatedSourceLength);
    Same(selected.qeph_metric().working_length_m(),resolution.source().canonical().data().inputs.units.length_to_m);
    ASSERT_EQ(selected.rows().size(),349645u);ASSERT_EQ(selected.counts().succeeded,349645u);EXPECT_EQ(selected.counts().rejected,0u);
    std::size_t q=0,t=0,b=0;
    for(std::size_t i=0;i<selected.rows().size();++i) {
        const auto& x=selected.rows()[i];const auto& y=legacy.rows()[i];
        ASSERT_EQ(x.element_id,y.element_id);ASSERT_EQ(x.family,y.family);ASSERT_EQ(x.status,y.status);
        EXPECT_EQ(x.role,y.role);EXPECT_EQ(x.rigid_root_index,y.rigid_root_index);EXPECT_EQ(x.canonical_parent,y.canonical_parent);
        if(const auto* ref=selected.qeph(i)){++q;auto normalized=*ref;Same(ref->input.projection_working_length_m,.001);
            // Only the immutable metric descriptor differs at startup; preserve
            // every reference coordinate/material/mass/inertia field exactly.
            normalized.input.projection_working_length_m=1.;Same(normalized,*legacy.qeph(i));}
        else if(const auto* ref=selected.t3(i)){++t;Same(*ref,*legacy.t3(i));}
        else {++b;ASSERT_NE(selected.qbat(i),nullptr);Same(selected.qbat(i)->quadrilateral(),legacy.qbat(i)->quadrilateral());
            SameInput(selected.qbat(i)->input().quadrilateral,legacy.qbat(i)->input().quadrilateral);}
    }
    EXPECT_EQ(q,324094u);EXPECT_EQ(t,21301u);EXPECT_EQ(b,4250u);
    const auto copied=selected;EXPECT_EQ(&copied.qeph_metric(),&selected.qeph_metric());
    const auto forecast=ForecastReferences(resolution,QephMetricProfile::AuthenticatedSourceLength,limits);
    auto exact=limits;exact.host_bytes=forecast.total_bytes;
    EXPECT_EQ(ForecastReferences(resolution,QephMetricProfile::AuthenticatedSourceLength,exact).total_bytes,forecast.total_bytes);
    --exact.host_bytes;EXPECT_THROW(VehicleShellReferences::Prepare(resolution,QephMetricProfile::AuthenticatedSourceLength,exact),std::exception);
    EXPECT_THROW(VehicleShellReferences::Prepare(resolution,static_cast<QephMetricProfile>(99),limits),std::exception);
    RecordProperty("qeph_source_metric_parents",std::to_string(q));RecordProperty("unchanged_t3_parents",std::to_string(t));
    RecordProperty("unchanged_legacy_qbat_parents",std::to_string(b));RecordProperty("forecast_bytes",std::to_string(forecast.total_bytes));
}
TEST(VehicleReferenceMetricSource, ExistingAuthenticatedAssemblyAdapterForwardsOnlyItsQephFamily) {
    namespace a=modelio::assembly;const auto source=a::test::section::Load(true);
    const a::SourceAssemblyShellInput legacy(source),explicit_legacy(source,QephMetricProfile::LegacyOneMetre),
        current(source,QephMetricProfile::AuthenticatedSourceLength);
    const auto before=legacy.input(),same=explicit_legacy.input(),after=current.input();
    ASSERT_EQ(before.qeph_count,after.qeph_count);ASSERT_EQ(before.t3_count,after.t3_count);
    EXPECT_EQ(current.qeph_metric().profile(),QephMetricProfile::AuthenticatedSourceLength);
    for(std::size_t i=0;i<before.qeph_count;++i){SameInput(before.qeph[i].reference,same.qeph[i].reference);
        auto fixed=after.qeph[i].reference;Same(fixed.projection_working_length_m,source.data().units.length_to_m);
        fixed.projection_working_length_m=1.;SameInput(fixed,before.qeph[i].reference);EXPECT_EQ(after.qeph[i].nodes,before.qeph[i].nodes);}
    for(std::size_t i=0;i<before.t3_count;++i)SameInput(before.t3[i].reference,after.t3[i].reference);
    const auto copied=current;Same(copied.qeph_metric().working_length_m(),.001);
    EXPECT_THROW(a::SourceAssemblyShellInput(source,static_cast<QephMetricProfile>(99)),std::exception);
}
}
