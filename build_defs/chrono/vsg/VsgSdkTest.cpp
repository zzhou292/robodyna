#include <gtest/gtest.h>
#include <vsg/core/Version.h>
#include <vsgXchange/Version.h>
#include <vsgImGui/RenderImGui.h>
#include <link.h>

#include <string>

TEST(VsgSdk, HeaderAndLinkedRuntimeVersionsAgreeWithoutOpeningADevice) {
    EXPECT_STREQ(vsgGetVersionString(), VSG_VERSION_STRING);
    EXPECT_STREQ(vsgXchangeGetVersionString(), VSGXCHANGE_VERSION_STRING);
    EXPECT_EQ(vsgBuiltAsSharedLibrary(), 1);
    EXPECT_EQ(vsgXchangeBuiltAsSharedLibrary(), 1);
    EXPECT_STREQ(ImGui::GetVersion(), IMGUI_VERSION);
    EXPECT_EQ(VK_HEADER_VERSION, 204);
}

TEST(VsgSdk, NoHistoricalChronoImplementationLibraryIsLoaded) {
    std::string forbidden;
    dl_iterate_phdr([](dl_phdr_info* info, std::size_t, void* data) {
        const std::string path = info->dlpi_name ? info->dlpi_name : "";
        if (path.find("libChrono_") != std::string::npos)
            *static_cast<std::string*>(data) = path;
        return 0;
    }, &forbidden);
    EXPECT_TRUE(forbidden.empty()) << forbidden;
}
