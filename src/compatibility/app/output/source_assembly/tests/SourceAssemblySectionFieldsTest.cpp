#include "SourceAssemblyOutputTestSupport.h"
#include "chrono/NodalCaptureLimits.h"
#include "output/source_assembly/SourceAssemblyOwnerScope.h"
#include <limits>

namespace crash::output::assembly::test {
TEST(SourceAssemblySectionFields, EverySourceParentAndAllThreePointsSerializeWithoutLoss) {
    const auto surface=Surface();Fields fields(surface);const auto doc=SectionFieldDocument(surface,fields.Q(),fields.T());
    ASSERT_EQ(doc["source_parents"].Size(),915u);ASSERT_EQ(doc["sections"].Size(),915u);
    ASSERT_EQ(doc["vertex_binding"].Size(),1030u);ASSERT_EQ(doc["triangle_binding"].Size(),1719u);
    EXPECT_STREQ(doc["source_inventory_sha256"].GetString(),source::PinnedYarisSixPartInventory().sha256.c_str());
    for(const auto& p:surface.parents()) {
        SCOPED_TRACE(p.source_index);const auto& map=doc["source_parents"][rapidjson::SizeType(p.source_index)];
        EXPECT_EQ(map[0].GetUint64(),p.source_index);EXPECT_EQ(map[1].GetUint64(),p.element);EXPECT_EQ(map[2].GetUint64(),p.part);
        EXPECT_EQ(map[3].GetUint64(),p.material);EXPECT_EQ(map[4].GetUint64(),p.section);EXPECT_EQ(map[5].GetUint64(),p.curve);
        EXPECT_EQ(map[6].GetUint64(),p.source_elform);EXPECT_STREQ(map[7].GetString(),p.family==source::ShellFamily::Qeph?"QEPH":"T3");
        EXPECT_EQ(map[8].GetUint64(),p.family_index);EXPECT_EQ(map[9].GetUint64(),p.first_triangle);EXPECT_EQ(map[10].GetUint64(),p.triangle_count);
        const auto view=p.family==source::ShellFamily::Qeph?fields.Q():fields.T();const auto& s=view.values[p.family_index];
        const auto& row=doc["sections"][rapidjson::SizeType(p.source_index)];const auto& d=s.diagnostics;
        const double expected[]{s.cumulative_plastic_work_J,d.plastic_work_density_increment,d.maximum_plastic_strain,
            d.mean_plastic_strain,d.minimum_tangent_ratio,d.mean_tangent_ratio,d.mean_yield_before_pa,d.last_point_yield_before_pa,
            view.reported_thickness_m[p.family_index]};
        ASSERT_EQ(row.Size(),10u);for(unsigned i=0;i<9;++i)EXPECT_EQ(Bits(row[i].GetDouble()),Bits(expected[i]));
        ASSERT_EQ(row[9].Size(),3u);
        for(unsigned layer=0;layer<3;++layer) {
            const auto& point=s.history.point[layer];const auto& values=row[9][layer];ASSERT_EQ(values.Size(),7u);
            for(unsigned c=0;c<5;++c)EXPECT_EQ(Bits(values[c].GetDouble()),Bits(point.stress[c]));
            EXPECT_EQ(Bits(values[5].GetDouble()),Bits(point.plastic_strain));EXPECT_EQ(Bits(values[6].GetDouble()),Bits(point.filtered_rate_per_s));
        }
    }
    for(unsigned i=0;i<3;++i) {
        EXPECT_EQ(Bits(doc["section_position_over_thickness"][i].GetDouble()),Bits(tl::fea::sections::LayerPosition(i)));
        EXPECT_EQ(Bits(doc["section_moment_weight"][i].GetDouble()),Bits(tl::fea::sections::LayerMomentWeight(i)));
    }
}
TEST(SourceAssemblySectionFields, LatePointFailureAndWrongCapacitiesCannotPublishPartialFields) {
    const auto surface=Surface();Fields fields(surface);std::vector<ReplayParentScalar> values(915);
    ASSERT_TRUE(CopyParentScalars(surface,fields.Q(),fields.T(),values));const auto before=values;
    const auto* poison=reinterpret_cast<const tl::fea::ShellBatchSectionState*>(std::uintptr_t{1});
    auto invalid=fields.T();invalid.count=std::numeric_limits<std::size_t>::max();
    auto q=fields.Q();q.values=poison;EXPECT_FALSE(CopyParentScalars(surface,q,invalid,values));
    invalid=fields.Q();--invalid.count;EXPECT_FALSE(CopyParentScalars(surface,invalid,fields.T(),values));
    fields.t.back().history.point[2].stress[4]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(CopyParentScalars(surface,fields.Q(),fields.T(),values));
    EXPECT_THROW(SectionFieldDocument(surface,fields.Q(),fields.T()),std::runtime_error);
    for(std::size_t i=0;i<values.size();++i) {
        EXPECT_EQ(values[i].source_parent,before[i].source_parent);EXPECT_EQ(values[i].value,before[i].value);
    }
    fields.t.back().history.point[2].stress[4]=0;fields.qt.back()=0;
    EXPECT_FALSE(ValidSections(surface,fields.Q(),fields.T()));
}
TEST(SourceAssemblyOwnerScope, OnlyInactiveOrExactCompleteRigidSourceAssociationIsAdmitted) {
    const tl::fea::NodalRigidGroupInfo expected{0x5941524953,6,76};
    EXPECT_TRUE(MatchesAssemblyRigidScope({},expected));EXPECT_TRUE(MatchesAssemblyRigidScope(expected,expected));
    for(const auto actual:{tl::fea::NodalRigidGroupInfo{expected.source_instance_id+1,6,76},
        {expected.source_instance_id,5,76},{expected.source_instance_id,6,75},{0,6,76},
        {expected.source_instance_id,0,76},{expected.source_instance_id,6,0},{expected.source_instance_id,0,0}})
        EXPECT_FALSE(MatchesAssemblyRigidScope(actual,expected));
    EXPECT_FALSE(MatchesAssemblyRigidScope(expected,{}));
}
TEST(NodalCaptureLimits, ExactActivePayloadAndOverflowRejectionsAreAtomic) {
    std::size_t bytes=9;EXPECT_TRUE(visual::NodalCaptureBytes(128,true,{},bytes));EXPECT_EQ(bytes,128u*26*sizeof(double));
    const auto legacy=bytes;EXPECT_FALSE(visual::NodalCaptureBytes(129,true,{},bytes));EXPECT_EQ(bytes,legacy);
    visual::NodalCaptureLimits limits{2048,512*1024};
    ASSERT_TRUE(visual::NodalCaptureBytes(1030,true,limits,bytes));EXPECT_EQ(bytes,1030u*26*sizeof(double));
    limits.max_host_bytes=bytes-1;const auto previous=bytes;
    EXPECT_FALSE(visual::NodalCaptureBytes(1030,true,limits,bytes));EXPECT_EQ(bytes,previous);
    limits.max_host_bytes=previous;EXPECT_TRUE(visual::NodalCaptureBytes(1030,true,limits,bytes));
    EXPECT_FALSE(visual::NodalCaptureBytes(std::numeric_limits<std::size_t>::max(),true,limits,bytes));EXPECT_EQ(bytes,previous);
    EXPECT_FALSE(visual::NodalCaptureBytes(1,false,{std::numeric_limits<std::size_t>::max(),512*1024},bytes));
    ASSERT_TRUE(visual::NodalCaptureBytes(1030,false,limits,bytes));EXPECT_EQ(bytes,1030u*12*sizeof(double));
}
} // namespace crash::output::assembly::test
