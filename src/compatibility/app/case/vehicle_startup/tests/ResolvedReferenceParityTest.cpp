#include "ResolvedReferenceSupport.h"
#include "modelio/source_assembly/SourceAssemblyShellInput.h"
#include <unordered_map>

namespace crash::cases::vehicle_startup::test {
TEST(VehicleResolvedReference, HistoricalSupportedReferencesAndActualV3BindingsRemainExact) {
    const auto& r=ResolvedReferences();
    const auto& legacy=References();
    ASSERT_EQ(r.rows().size(),legacy.rows().size());
    EXPECT_EQ(legacy.resolution(),nullptr);
    EXPECT_EQ(legacy.counts().attempted,278301);
    EXPECT_EQ(legacy.counts().unresolved,71344);
    std::unordered_map<std::uint64_t,std::size_t> rows;
    rows.reserve(r.rows().size());
    for (std::size_t e=0;e<r.rows().size();++e) {
        const auto& a=r.rows()[e];
        const auto& b=legacy.rows()[e];
        ASSERT_EQ(a.element_id,b.element_id);
        ASSERT_EQ(a.part_id,b.part_id);
        ASSERT_EQ(a.material_id,b.material_id);
        ASSERT_EQ(a.section_id,b.section_id);
        ASSERT_EQ(a.source_line,b.source_line);
        rows.emplace(a.element_id,e);
        if (b.status==ReferenceStatus::UnresolvedDeclaration) continue;
        ASSERT_EQ(a.status,b.status);
        ASSERT_EQ(a.family,b.family);
        if (const auto* q=legacy.qeph(e)) {
            ASSERT_NE(r.qeph(e),nullptr);
            Same(*q,*r.qeph(e));
        }
        if (const auto* t=legacy.t3(e)) {
            ASSERT_NE(r.t3(e),nullptr);
            Same(*t,*r.t3(e));
        }
    }
    for (bool mixed : {false,true}) {
        const auto* path=std::getenv(mixed ? "ROBO_VEHICLE_MIXED" : "ROBO_VEHICLE_ELASTIC");
        ASSERT_NE(path,nullptr);
        const auto assembly=modelio::assembly::SourceAssembly::Read(path,modelio::assembly::test::section::Identity(mixed));
        const modelio::assembly::SourceAssemblyShellInput input(assembly);
        tl::fea::ShellBatchBinding binding;
        ASSERT_EQ(binding.Initialize(input.input(),{}).status,tl::fea::ShellBindingStatus::Success);
        for (const auto& parent : assembly.data().parents) {
            const auto found=rows.find(parent.source_id);
            ASSERT_NE(found,rows.end());
            if (parent.family==modelio::assembly::ShellFamily::Qeph) {
                ASSERT_NE(r.qeph(found->second),nullptr);
                Same(*r.qeph(found->second),binding.qeph_reference(parent.family_index));
            } else {
                ASSERT_NE(r.t3(found->second),nullptr);
                Same(*r.t3(found->second),binding.t3_reference(parent.family_index));
            }
        }
    }
}

TEST(VehicleResolvedReference, ExplicitProfileAndLateBudgetFailurePreserveImmutableAssessment) {
    const auto& r=ResolvedReferences();
    const auto& resolution=source_test::Resolution();
    const auto* rows=r.rows().data();
    const auto original=r.rows().back();
    auto copied=r;
    auto moved=std::move(copied);
    EXPECT_EQ(copied.rows().data(),rows);
    EXPECT_EQ(moved.rows().data(),rows);
    EXPECT_EQ(moved.resolution()->parents().data(),resolution.parents().data());
    EXPECT_GT(r.forecast().total_bytes,512*1024*1024);
    for (unsigned fault=0;fault<8;++fault) {
        auto limits=ReferenceLimits::ResolvedSections();
        switch (fault) {
        case 0: limits=ReferenceLimits{}; break;
        case 1: limits.host_bytes=r.forecast().total_bytes-1; break;
        case 2: limits.parents=349644; break;
        case 3: limits.nodes=359784; break;
        case 4: limits.host_bytes=768*1024*1024+1; break;
        case 5: limits.profile=static_cast<ReferenceProfile>(77); break;
        case 6: limits.profile=ReferenceProfile::Legacy; break;
        case 7: limits.parents=524289; break;
        }
        EXPECT_THROW(VehicleShellReferences::Prepare(resolution,limits),std::runtime_error);
        EXPECT_EQ(r.rows().data(),rows);
        EXPECT_EQ(r.rows().back().element_id,original.element_id);
        EXPECT_EQ(r.rows().back().status,original.status);
    }
    auto legacy_limits=ReferenceLimits{};
    legacy_limits.host_bytes=512*1024*1024+1;
    EXPECT_THROW(ForecastReferences(resolution.source(),legacy_limits),std::runtime_error);
    EXPECT_THROW(ForecastReferences(resolution.source(),ReferenceLimits::ResolvedSections()),std::runtime_error);
    auto exact=ReferenceLimits::ResolvedSections();
    exact.host_bytes=r.forecast().total_bytes;
    exact.parents=r.rows().size();
    exact.nodes=r.source().counts().nodes;
    EXPECT_EQ(ForecastReferences(resolution,exact).total_bytes,r.forecast().total_bytes);
    const auto retry=VehicleShellReferences::Prepare(resolution,exact);
    ASSERT_EQ(retry.rows().size(),r.rows().size());
    EXPECT_EQ(retry.counts().status,r.counts().status);
    for (std::size_t e=0;e<r.rows().size();++e) {
        ASSERT_EQ(retry.rows()[e].element_id,r.rows()[e].element_id);
        ASSERT_EQ(retry.rows()[e].status,r.rows()[e].status);
        if (const auto* q=r.qeph(e)) Same(*q,*retry.qeph(e));
        if (const auto* t=r.t3(e)) Same(*t,*retry.t3(e));
    }
}
} // namespace crash::cases::vehicle_startup::test
