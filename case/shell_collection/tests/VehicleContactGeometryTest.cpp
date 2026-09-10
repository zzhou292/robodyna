#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include "lib_utest/qualification/vehicle_shell_host/VehicleShellFixture.h"
#include <gtest/gtest.h>

namespace crash::cases::test {
namespace fe=tl::fea;
namespace sc=tlfea::contact;
namespace {
void CheckComplete(const ShellCollectionContactGeometry& geometry,const fe::ShellBatchBinding& binding) {
    ASSERT_TRUE(geometry.prepared());ASSERT_NE(geometry.weights(),nullptr);
    const auto& weights=*geometry.weights();
    ASSERT_EQ(weights.parent_count(),binding.qeph_count()+binding.t3_count());
    ASSERT_EQ(weights.node_count(),binding.node_count());
    EXPECT_EQ(geometry.binding()->inventory(),binding.inventory());
    for(std::size_t n=0;n<binding.node_count();++n) {
        const auto x=geometry.positions().at(n);
        const auto expected=binding.nodes()[n].position;
        EXPECT_EQ(x.x,expected.x);EXPECT_EQ(x.y,expected.y);EXPECT_EQ(x.z,expected.z);
        EXPECT_EQ(weights.node(n).node,n);
    }
    for(std::size_t p=0;p<weights.parent_count();++p) {
        const auto* key=geometry.parent_from_weight(p);ASSERT_NE(key,nullptr);
        EXPECT_EQ(key->source_id,weights.parent(p).parent_element_id);
        const bool quad=key->family==fe::ShellBindingFamily::Qeph;
        EXPECT_EQ(key->source_id,quad?binding.qeph_source_id(key->family_index):binding.t3_source_id(key->family_index));
        EXPECT_EQ(weights.parent(p).arity,quad?4u:3u);
        for(unsigned l=0;l<weights.parent(p).arity;++l)
            EXPECT_EQ(weights.parent(p).nodes[l],quad?binding.qeph_nodes(key->family_index)[l]:binding.t3_nodes(key->family_index)[l]);
    }
}
}
TEST(VehicleContactGeometry, ExplicitLimitsPreserveLegacyRejectAndBoundedRetry) {
    vehicle_shell_test::Fixture source;fe::ShellBatchBinding binding;
    ASSERT_EQ(binding.Initialize(source.input(),fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
    ShellCollectionContactGeometry geometry;
    EXPECT_EQ(geometry.Initialize(binding).status,ShellContactGeometryStatus::ResourceLimit);
    auto malformed=ShellContactGeometryLimits::Vehicle();++malformed.weights.max_nodes;
    EXPECT_EQ(geometry.Initialize(binding,malformed).status,ShellContactGeometryStatus::ResourceLimit);
    malformed=ShellContactGeometryLimits::Vehicle();++malformed.weights.max_parents;
    EXPECT_EQ(geometry.Initialize(binding,malformed).status,ShellContactGeometryStatus::ResourceLimit);
    malformed=ShellContactGeometryLimits::Vehicle();
    malformed.weights.profile=static_cast<sc::NodalWallWeightProfile>(-1);
    EXPECT_EQ(geometry.Initialize(binding,malformed).status,ShellContactGeometryStatus::InvalidInput);
    auto limits=ShellContactGeometryLimits::Vehicle();limits.max_startup_bytes=1;
    EXPECT_EQ(geometry.Initialize(binding,limits).status,ShellContactGeometryStatus::ResourceLimit);
    EXPECT_FALSE(geometry.prepared());limits=ShellContactGeometryLimits::Vehicle();
    ASSERT_TRUE(geometry.Initialize(binding,limits));ASSERT_NO_FATAL_FAILURE(CheckComplete(geometry,binding));
    ShellCollectionContactGeometry exact;limits.max_startup_bytes=geometry.startup_payload_bytes();
    ASSERT_TRUE(exact.Initialize(binding,limits));ASSERT_NO_FATAL_FAILURE(CheckComplete(exact,binding));
    ShellCollectionContactGeometry short_budget;--limits.max_startup_bytes;
    EXPECT_EQ(short_budget.Initialize(binding,limits).status,ShellContactGeometryStatus::ResourceLimit);
    const auto* weights=geometry.weights();
    EXPECT_EQ(geometry.Initialize(binding).status,ShellContactGeometryStatus::InvalidInput);EXPECT_EQ(geometry.weights(),weights);
}
TEST(VehicleContactGeometry, SourceSizedMixedGeometryRetainsEveryOriginalBindingEntry) {
    using F=vehicle_shell_test::Fixture;
    auto source=std::make_unique<F>(F::SourceQ,F::SourceT,F::SourceNodes);fe::ShellBatchBinding binding;
    ASSERT_EQ(binding.Initialize(source->input(),fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
    source.reset();ShellCollectionContactGeometry geometry;
    const auto report=geometry.Initialize(binding,ShellContactGeometryLimits::Vehicle());
    ASSERT_TRUE(report)<<report.message;ASSERT_NO_FATAL_FAILURE(CheckComplete(geometry,binding));
    EXPECT_EQ(geometry.parent_from_weight(F::SourceQ+F::SourceT),nullptr);
    RecordProperty("scope","synthetic native Q/T binding at no-tire source counts; immutable contact geometry only");
    RecordProperty("startup_payload_reservation_bytes",std::to_string(geometry.startup_payload_bytes()));
    RecordProperty("retained_weight_bytes",std::to_string(geometry.weights()->owned_payload_bytes()));
}
} // namespace crash::cases::test
