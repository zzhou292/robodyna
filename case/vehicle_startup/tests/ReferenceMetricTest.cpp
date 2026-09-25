#include "../ReferenceStorage.h"
#include "../QbatReferenceInput.h"
#include "ReferenceComparison.h"
#include "modelio/vehicle_sections/tests/MidlayerFixture.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "lib_src/elements/ShellBatchBinding.h"
#include <limits>
namespace crash::cases::vehicle_startup::test {
namespace a=modelio::assembly;
namespace {
const a::SourceReferenceNode Nodes[4]{{7,{-0.,0,0}},{3,{.01,0,0}},{11,{.01,.008,.00001}},{13,{0,.008,0}}};
a::Material Material(){a::Material m;m.density_kg_m3=7850;m.young_pa=210e9;m.poisson_ratio=.3;return m;}
a::Section Section(){a::Section s;s.thickness_m.fill(.001);return s;}
auto Quad(){return a::PackShellReference<tl::fea::qeph::ReferenceInput>(Nodes,Material(),Section());}
auto Triangle(){const a::SourceReferenceNode nodes[3]{Nodes[0],Nodes[2],Nodes[3]};return a::PackShellReference<tl::fea::t3::ReferenceInput>(nodes,Material(),Section());}
}
TEST(VehicleReferenceMetric, LegacyDefaultAndExplicitSourceMetreKeepEveryInputBit) {
    const auto original=Quad();const auto legacy=a::QephReferenceMetric::Resolve(QephMetricProfile::LegacyOneMetre,{1000,.001,1});
    const auto metre=a::QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,{1,1,1});
    SameInput(a::WithQephMetric(original,legacy),original);SameInput(a::WithQephMetric(original,metre),original);
    EXPECT_EQ(legacy.profile(),QephMetricProfile::LegacyOneMetre);EXPECT_EQ(metre.profile(),QephMetricProfile::AuthenticatedSourceLength);
    tl::fea::qeph::ReferenceData expected,actual;
    ASSERT_EQ(tl::fea::qeph::InitializeReference(original,expected),tl::fea::qeph::Status::kSuccess);
    ASSERT_EQ(tl::fea::qeph::InitializeReference(a::WithQephMetric(original,legacy),actual),tl::fea::qeph::Status::kSuccess);
    Same(expected,actual);
}
TEST(VehicleReferenceMetric, NonMillimetreUnitsForwardExactlyWithoutRescalingPhysicalInputs) {
    const auto original=Quad();
    for(double length:{.001,.01,.0254,1.,1000.}) {
        const auto metric=a::QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,{2.5,length,.5});
        auto input=a::WithQephMetric(original,metric);Same(input.projection_working_length_m,length);
        input.projection_working_length_m=1;SameInput(input,original);
        tl::fea::qeph::ReferenceData reference;
        ASSERT_EQ(tl::fea::qeph::InitializeReference(a::WithQephMetric(original,metric),reference),tl::fea::qeph::Status::kSuccess);
        Same(reference.input.projection_working_length_m,length);Same(reference.input.position,original.position);
    }
}
TEST(VehicleReferenceMetric, UnknownOrIncompleteOptInUnitsRejectBeforeReplacingInput) {
    const auto original=Quad();auto visible=original;
    for(auto units:{a::SourceUnits{},a::SourceUnits{0,.001,1},a::SourceUnits{1,0,1},a::SourceUnits{1,.001,0},
        a::SourceUnits{-1,.001,1},a::SourceUnits{1,-.001,1},a::SourceUnits{1,.001,-1},
        a::SourceUnits{1,std::numeric_limits<double>::infinity(),1},a::SourceUnits{1,std::numeric_limits<double>::quiet_NaN(),1}}) {
        EXPECT_THROW(visible=a::WithQephMetric(original,a::QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,units)),std::exception);
        SameInput(visible,original);
    }
    EXPECT_THROW(a::QephReferenceMetric::Resolve(static_cast<QephMetricProfile>(99),{1,1,1}),std::exception);
    // Legacy behavior still imposes its existing source-reader validation; the
    // numerical1m packing path does not infer a metric from missing units.
    Same(a::QephReferenceMetric::Resolve(QephMetricProfile::LegacyOneMetre,{}).working_length_m(),1.);
}
TEST(VehicleReferenceMetric, MixedQephTriangleAndQbatKeepDistinctNumericalFamilies) {
    const auto metric=a::QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,{1000,.001,1});
    const auto declaration=modelio::vehicle::resolution::ReadMidlayer(modelio::vehicle::test::MidlayerFields(),{1000,.001,1});
    const auto qbat_quad=a::PackShellReference<tl::fea::qeph::ReferenceInput>(Nodes,declaration.material,declaration.section);
    const auto native=a::detail::NativeMaterial(declaration.material,a::detail::NativeLaw44Rate::FilteredZeroC);
    const auto qbat=detail::OriginalMidlayerQbatInput(qbat_quad,native);const auto triangle=Triangle();
    detail::ReferenceStorage mixed;ReferenceRow row;row.element_id=101;
    detail::Append(mixed,row,a::WithQephMetric(Quad(),metric));row.element_id=102;detail::Append(mixed,row,triangle);
    row.element_id=103;detail::Append(mixed,row,qbat);
    ASSERT_EQ(mixed.counts.succeeded,3u);ASSERT_EQ(mixed.qeph.size(),1u);ASSERT_EQ(mixed.t3.size(),1u);ASSERT_EQ(mixed.qbat.size(),1u);
    Same(mixed.qeph[0].input.projection_working_length_m,.001);SameInput(mixed.t3[0].input,triangle);
    Same(mixed.qbat[0].quadrilateral().input.projection_working_length_m,1.);SameInput(mixed.qbat[0].input().quadrilateral,qbat.quadrilateral);
}
TEST(VehicleReferenceMetric, NativeHistoryAndCompleteBindingIdentityRetainWorkingMetric) {
    namespace f=tl::fea;const auto metric=a::QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,{1000,.001,1});
    f::qeph::ReferenceData legacy,native;
    ASSERT_EQ(f::qeph::InitializeReference(Quad(),legacy),f::qeph::Status::kSuccess);
    ASSERT_EQ(f::qeph::InitializeReference(a::WithQephMetric(Quad(),metric),native),f::qeph::Status::kSuccess);
    f::qeph::History history;ASSERT_EQ(f::qeph::InitializeHistory(native,{},history),f::qeph::Status::kSuccess);
    EXPECT_TRUE(history.matches_reference(native));EXPECT_FALSE(history.matches_reference(legacy));
    f::ShellQephBindingInput q{legacy.input,{0,1,2,3},101};f::ShellT3BindingInput t{Triangle(),{0,2,3},102};
    f::ShellBatchBinding before,after;ASSERT_EQ(before.Initialize({&q,&t,1,1,4}).status,f::ShellBindingStatus::Success);
    q.reference=native.input;ASSERT_EQ(after.Initialize({&q,&t,1,1,4}).status,f::ShellBindingStatus::Success);
    EXPECT_NE(before.inventory(),after.inventory());Same(after.qeph_reference(0).input.projection_working_length_m,.001);
    auto retained=after;EXPECT_EQ(retained.inventory(),after.inventory());Same(retained.qeph_reference(0).input.projection_working_length_m,.001);
}
TEST(VehicleReferenceMetric, NativeStartupOwnsRepresentabilityAndPreservesOutputOnExtremePositiveLengths) {
    namespace q=tl::fea::qeph;q::ReferenceData visible,expected;
    ASSERT_EQ(q::InitializeReference(Quad(),visible),q::Status::kSuccess);expected=visible;
    detail::ReferenceStorage assessment;
    for(double length:{1e-300,1e300}) {
        const auto metric=a::QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,{1,length,1});
        Same(metric.working_length_m(),length);const auto input=a::WithQephMetric(Quad(),metric);
        EXPECT_EQ(q::InitializeReference(input,visible),q::Status::kInvalidInput);Same(visible,expected);
        detail::Append(assessment,{},input);
    }
    EXPECT_EQ(assessment.counts.rejected,2u);EXPECT_TRUE(assessment.qeph.empty());
    for(const auto& row:assessment.rows)EXPECT_EQ(row.status,ReferenceStatus::InvalidInput);
}

}
