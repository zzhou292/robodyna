#include "ReplayPartColorTestSupport.h"
#include "ReplayColorMetadata.h"

namespace {
using namespace replay_color_test;
using namespace crash::visual;
TEST(ReplayColorMetadata, PartManifestCarriesCompleteOriginalIdPaletteWithoutPlasticScale) {
    AcceptedReplayScene scene;ASSERT_EQ(Initialize(scene).status,ReplaySceneStatus::Ok);
    crash::output::Document document;document.SetObject();
    crash::viewer::AppendReplayColorMetadata(document,Info(),scene);
    EXPECT_STREQ(document["surface_color_mode"].GetString(),"part-id");
    EXPECT_STREQ(document["surface_color_palette"].GetString(),ReplayPartPaletteName);
    EXPECT_EQ(document["surface_color_palette_seed"].GetUint64(),ReplayPartPaletteSeed);
    ASSERT_EQ(document["surface_color_legend"].Size(),2u);
    EXPECT_EQ(document["surface_color_legend"][0][0].GetUint64(),2000145u);
    EXPECT_EQ(document["surface_color_legend"][1][0].GetUint64(),2000157u);
    const auto& legend=*scene.part_legend();
    for(rapidjson::SizeType i=0;i<2;++i) {
        const auto& row=document["surface_color_legend"][i];
        EXPECT_EQ(row[1].GetDouble(),legend[i].color.R);EXPECT_EQ(row[2].GetDouble(),legend[i].color.G);
        EXPECT_EQ(row[3].GetDouble(),legend[i].color.B);
    }
    EXPECT_FALSE(document.HasMember("surface_color_min"));EXPECT_FALSE(document.HasMember("surface_color_max"));
}
TEST(ReplayColorMetadata, PlasticAndUniformModesKeepTheirActualQuantity) {
    for(auto mode:{ReplayColorMode::Automatic,ReplayColorMode::Uniform}) {
        AcceptedReplayScene scene;ASSERT_EQ(Initialize(scene,mode).status,ReplaySceneStatus::Ok);
        crash::output::Document document;document.SetObject();
        crash::viewer::AppendReplayColorMetadata(document,Info(),scene);
        EXPECT_STREQ(document["surface_color_mode"].GetString(),ReplayColorModeName(scene.color_mode()));
        EXPECT_FALSE(document.HasMember("surface_color_legend"));
        if(mode==ReplayColorMode::Automatic)EXPECT_EQ(document["surface_color_max"].GetDouble(),.02);
        else EXPECT_FALSE(document.HasMember("surface_color_max"));
    }
    AcceptedReplayScene empty;crash::output::Document document;document.SetObject();
    EXPECT_THROW(crash::viewer::AppendReplayColorMetadata(document,Info(),empty),std::runtime_error);
    EXPECT_TRUE(document.ObjectEmpty());
}
} // namespace
