#include "ComponentFrameTestSupport.h"
#include <limits>

namespace crash::output::assembly::binary::test {
TEST(ComponentBinaryValues, ActualSourceContextRetainsFullNativeMappingAndRunIndependentDigest) {
    fixture::WallFields f;
    const auto c=ComponentContext(f.bindings,f.surface,1,2,f.stamp.fixed_dt);
    EXPECT_EQ(c.nodes(),1030u);EXPECT_EQ(c.parents().size(),915u);EXPECT_EQ(c.points(),2745u);
    const auto alternate=SourceAssemblySurface::Prepare(f.bindings.source(),{991,992,993},99,f.bindings.source_instance_id());
    EXPECT_EQ(ComponentMappingDigest(f.surface),ComponentMappingDigest(alternate));
    EXPECT_EQ(c.identity().source_inventory_sha256,f.bindings.source().data().identity.sha256);
    EXPECT_EQ(c.identity().source_mapping_sha256,ComponentMappingDigest(f.surface));
    records::RecordLimits limits;limits.points=c.points()-1;
    EXPECT_THROW(ComponentContext(f.bindings,f.surface,1,2,f.stamp.fixed_dt,limits),std::runtime_error);
    for(std::size_t i=0;i<c.parents().size();++i) {
        EXPECT_EQ(c.parents()[i].source_element,f.bindings.source().data().parents[i].source_id);
        EXPECT_EQ(c.parents()[i].source_part,f.bindings.source().data().parents[i].part_id);
        EXPECT_EQ(c.point_offsets()[i],3*i);
    }
}
TEST(ComponentBinaryValues, SyntheticFormattingMatchesEveryOldPositionPointAndExactStamp) {
    fixture::WallFields f;const auto& settings=*f.setup.settings();
    const auto c=ComponentContext(f.bindings,f.surface,settings.configuration_id,settings.qualification_id,f.stamp.fixed_dt);
    auto values=Storage(c);detail::StageFrame(c,f.View(),values);
    CompareLegacy(c,values,wall_fields::FrameDocument(f.View()));
    f.Interval();f.x.back()=std::nextafter(f.x.back(),std::numeric_limits<double>::infinity());
    detail::StageFrame(c,f.View(),values);CompareLegacy(c,values,wall_fields::FrameDocument(f.View()));
    fixture::Directory dir;ASSERT_TRUE(std::filesystem::create_directory(dir.path));
    const auto file=records::WriteFrame(dir.path,"component",c,{values.stamp,values.position_xyz.data(),values.position_xyz.size(),values.plastic_points.data(),values.plastic_points.size()});
    Same(values,records::ReadFrame(dir.path,c,file,values.stamp));
    const auto held=values;auto wrong=values.stamp;++wrong.attempt;
    EXPECT_THROW(records::ReadFrame(dir.path,c,file,wrong),std::runtime_error);Same(held,values);
}
TEST(ComponentBinaryValues, LatePointPhaseAndCapacityFailuresPreservePublishedStorage) {
    fixture::WallFields f;f.Interval();const auto& settings=*f.setup.settings();
    const auto c=ComponentContext(f.bindings,f.surface,settings.configuration_id,settings.qualification_id,f.stamp.fixed_dt);
    auto values=Storage(c);detail::StageFrame(c,f.View(),values);const auto held=values;
    const auto attempt=f.c.attempt;++f.c.attempt;
    EXPECT_THROW(detail::StageFrame(c,f.View(),values),std::runtime_error);Same(held,values);f.c.attempt=attempt;
    auto& last=f.sections.t.back().history.point[2].plastic_strain;const auto saved=last;
    last=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(detail::StageFrame(c,f.View(),values),std::runtime_error);Same(held,values);last=saved;
    auto wrong_parents=c.parents();++wrong_parents.back().source_part;
    const auto wrong_context=records::Context::Create(c.identity(),c.nodes(),wrong_parents.data(),wrong_parents.size(),c.fixed_dt());
    EXPECT_THROW(detail::StageFrame(wrong_context,f.View(),values),std::runtime_error);Same(held,values);
    auto short_values=held;short_values.plastic_points.pop_back();const auto short_before=short_values;
    EXPECT_THROW(detail::StageFrame(c,f.View(),short_values),std::runtime_error);Same(short_before,short_values);
    detail::StageFrame(c,f.View(),values);Same(held,values);
}
} // namespace crash::output::assembly::binary::test
