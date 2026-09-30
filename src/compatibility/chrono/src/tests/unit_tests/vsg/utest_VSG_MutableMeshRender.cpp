// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2026 projectchrono.org
// Use of this source code is governed by the BSD-style license in LICENSE.
// =============================================================================
#include <gtest/gtest.h>
#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemNSC.h"
#include <cstdlib>
#include <filesystem>

namespace {
struct Pixels { std::size_t red=0, green=0; };
Pixels Inspect(const std::filesystem::path& path) {
    auto options=vsg::Options::create();
    options->add(vsgXchange::all::create());
    auto data=vsg::read_cast<vsg::Data>(path.string(),options);
    if(!data || (data->valueSize()!=3 && data->valueSize()!=4))
        throw std::runtime_error("Expected decoded RGB/RGBA qualification screenshot");
    const auto count=std::size_t(data->width())*data->height();
    const auto stride=data->valueSize();
    if(data->dataSize()!=count*stride)
        throw std::runtime_error("Screenshot shape/bytes differ");
    const auto* values=static_cast<const unsigned char*>(data->dataPointer());
    Pixels out;
    for(std::size_t i=0;i<count;++i) {
        const int r=values[stride*i],g=values[stride*i+1],b=values[stride*i+2];
        out.red += r>64 && r>g+32 && r>b+32;
        out.green += g>64 && g>r+32 && g>b+32;
    }
    return out;
}
}

TEST(VSGMutableMeshRender, ActualColoredFacesDisappearAndReturnWithoutRebinding) {
    // Explicit opt-in: normal owning host tests open no GUI or GPU device.
    const char* output=std::getenv("CHRONO_VSG_MUTABLE_RENDER_OUTPUT");
    if(!output || !*output) GTEST_SKIP() << "Actual render requires a new output directory and guarded GPU invocation";
    const std::filesystem::path directory(output);
    ASSERT_FALSE(std::filesystem::exists(directory));
    ASSERT_TRUE(std::filesystem::create_directory(directory));
    chrono::SetChronoDataPath(CHRONO_VSG_TEST_DATA_DIR);
    auto mesh=std::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->GetCoordsVertices()={{-1,-.5,0},{-.1,-.5,0},{-.55,.5,0},
                              {.1,-.5,0},{1,-.5,0},{.55,.5,0}};
    const std::vector<chrono::ChVector3i> faces{{0,1,2},{3,4,5}};
    const std::vector<chrono::ChVector3i> colors{{0,0,0},{1,1,1}};
    mesh->GetIndicesVertices()=faces;
    mesh->GetCoordsColors()={{1,0,0},{0,1,0}};
    mesh->GetIndicesColors()=colors;
    auto shape=std::make_shared<chrono::ChVisualShapeTriangleMesh>();
    shape->SetMesh(mesh,false);
    shape->SetMutable(true);
    shape->SetDoubleFaced(true);
    shape->SetBackfaceCull(false);
    ASSERT_FALSE(shape->IsFixedConnectivity());
    auto carrier=std::make_shared<chrono::ChBody>();
    carrier->SetFixed(true);
    carrier->EnableCollision(false);
    carrier->AddVisualShape(shape);
    chrono::ChSystemNSC system;
    system.AddBody(carrier);
    chrono::vsg3d::ChVisualSystemVSG visual;
    visual.AttachSystem(&system);
    visual.SetLoadingThreadCount(1);
    visual.SetWindowSize(640,480);
    visual.SetWindowTitle("robo-dyna renderer qualification | synthetic faces");
    visual.SetCameraVertical(chrono::CameraVerticalDir::Y);
    visual.AddCamera({0,0,3},{0,0,0});
    visual.SetCameraAngleDeg(45);
    visual.SetBackgroundColor({.06f,.08f,.11f});
    visual.SetBaseGuiVisibility(false);
    visual.HideLogo();
    visual.SetTargetRenderFPS(0);
    visual.Initialize();
    ASSERT_TRUE(visual.IsInitialized());
    const auto device=visual.GetWindow()->getPhysicalDevice()->getProperties();
    RecordProperty("vulkan_device_name",device.deviceName);
    RecordProperty("vulkan_vendor_id",std::to_string(device.vendorID));
    ImGui::GetIO().IniFilename=nullptr;
    std::vector<Pixels> screenshots;
    for(std::size_t count:{2u,1u,0u,2u}) {
        mesh->GetIndicesVertices().assign(faces.begin(),faces.begin()+count);
        mesh->GetIndicesColors().assign(colors.begin(),colors.begin()+count);
        const auto path=directory/("faces-"+std::to_string(screenshots.size())+".png");
        // ExportScreenImage reads the previous swapchain image. Present this
        // same geometry first, then request its capture without advancing it.
        ASSERT_TRUE(visual.Run());
        visual.BeginScene();visual.Render();visual.EndScene();
        ASSERT_TRUE(visual.Run());
        visual.WriteImageToFile(path.string());
        visual.BeginScene();visual.Render();visual.EndScene();
        ASSERT_TRUE(std::filesystem::is_regular_file(path));
        screenshots.push_back(Inspect(path));
    }
    visual.Quit();
    ASSERT_EQ(screenshots.size(),4u);
    EXPECT_GT(screenshots[0].red,100u);EXPECT_GT(screenshots[0].green,100u);
    EXPECT_GT(screenshots[1].red,100u);EXPECT_EQ(screenshots[1].green,0u);
    EXPECT_EQ(screenshots[2].red,0u);EXPECT_EQ(screenshots[2].green,0u);
    EXPECT_GT(screenshots[3].red,100u);EXPECT_GT(screenshots[3].green,100u);
    EXPECT_EQ(shape->GetMesh(),mesh);
    EXPECT_EQ(system.GetChTime(),0);
}
