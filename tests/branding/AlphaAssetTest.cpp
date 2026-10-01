#include "chrono_thirdparty/stb/stb_image.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::array<std::string, 2> asset_paths;

std::vector<unsigned char> ReadAsset(const std::string& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
        throw std::runtime_error("Cannot read declared brand asset: " + path);
    const auto length = input.tellg();
    if (length < 8 || length > 16 * 1024 * 1024)
        throw std::runtime_error("Brand PNG is outside the bounded input size: " + path);
    std::vector<unsigned char> bytes(static_cast<std::size_t>(length));
    input.seekg(0);
    if (!input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error("Incomplete brand PNG read: " + path);
    return bytes;
}

void ExpectTransparentRgbaPng(const std::vector<unsigned char>& bytes) {
    constexpr std::array<unsigned char, 8> png_magic{137, 80, 78, 71, 13, 10, 26, 10};
    ASSERT_TRUE(std::equal(png_magic.begin(), png_magic.end(), bytes.begin()));
    int width = 0, height = 0, channels = 0;
    ASSERT_EQ(stbi_info_from_memory(bytes.data(), static_cast<int>(bytes.size()), &width, &height, &channels), 1);
    ASSERT_GT(width, 0);
    ASSERT_GT(height, 0);
    ASSERT_EQ(channels, 4) << "The real source must contain alpha, not synthesized decoder alpha";
    const auto pixels_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    ASSERT_LE(pixels_count, 16u * 1024u * 1024u);
    std::unique_ptr<stbi_uc, void (*)(void*)> pixels(
        stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &width, &height, &channels, 4),
        stbi_image_free);
    ASSERT_NE(pixels, nullptr) << stbi_failure_reason();
    bool transparent = false;
    bool opaque = false;
    for (std::size_t i = 0; i < pixels_count; ++i) {
        const auto alpha = pixels.get()[4 * i + 3];
        transparent = transparent || alpha == 0;
        opaque = opaque || alpha == 255;
    }
    EXPECT_TRUE(transparent) << "Logo background must contain fully transparent pixels";
    EXPECT_TRUE(opaque) << "Logo artwork must contain opaque pixels";
}

TEST(BrandAssets, MasterAndRuntimeAreIdenticalRealTransparentPngs) {
    const auto master = ReadAsset(asset_paths[0]);
    const auto runtime = ReadAsset(asset_paths[1]);
    ASSERT_EQ(master, runtime) << "Runtime logo must preserve the exact master asset";
    {
        SCOPED_TRACE("master");
        ExpectTransparentRgbaPng(master);
    }
    {
        SCOPED_TRACE("runtime");
        ExpectTransparentRgbaPng(runtime);
    }
}
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 3) {
        std::cerr << "Expected the declared master and runtime PNG paths\n";
        return 2;
    }
    asset_paths = {argv[1], argv[2]};
    return RUN_ALL_TESTS();
}
