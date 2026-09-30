#include "FieldsFixture.h"
#include <gtest/gtest.h>
namespace crash::output::physical_frames::test {
TEST(PhysicalCaptureValues, GlobalLaw1HasZeroPointsAndNoFabricatedLayerAccess) {
    records::ParentPoints parent{101,10,2,QephFamily,0,records::PlasticField::NotApplicable};
    const auto context=records::Context::Create(records::test::Id(),3,&parent,1,.125);
    std::vector<ParentField> mapping{{QephFamily,0,L::GlobalLaw1Npt0}};
    detail::FrameBuffers buffers(context);
    f::ShellBatchLayeredSection row=f::ShellBatchLayeredSection::GlobalLaw1();std::uint8_t active=1;
    EXPECT_EQ(row.elastic(),nullptr);EXPECT_EQ(row.plastic(),nullptr);EXPECT_EQ(row.one_point(),nullptr);
    detail::StageLayered(context,mapping,QephFamily,&row,&active,1,buffers.Staging(),buffers.flags);
    buffers.Finish(context,{});
    EXPECT_EQ(buffers.frames[buffers.selected].plastic_points.size(),0u);
    EXPECT_FALSE(records::ParentPlasticMaxima(context,buffers.frames[buffers.selected])[0]);
    EXPECT_TRUE(buffers.activity[buffers.selected]->active(0));
    const auto old=buffers.selected;
    auto wrong=parent;wrong.native_points=3;
    const auto corrupt=records::Context::Create(records::test::Id(),3,&wrong,1,.125);
    EXPECT_THROW(detail::StageLayered(corrupt,mapping,QephFamily,&row,&active,1,buffers.Staging(),buffers.flags),std::exception);
    EXPECT_EQ(buffers.selected,old);
    row=f::ShellBatchLayeredSection::Elastic({});
    EXPECT_THROW(detail::StageLayered(context,mapping,QephFamily,&row,&active,1,buffers.Staging(),buffers.flags),std::exception);
    EXPECT_EQ(buffers.selected,old);
}
}
