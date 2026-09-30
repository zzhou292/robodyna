#include "SourcePartMaterial.h"
#include "output/ArtifactIO.h"
#include "lib_utest/qualification/native/law44/YarisCurveFixture.h"
#include <gtest/gtest.h>
#include <array>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <vector>

namespace crash::cases::source_part_plastic {
namespace {
namespace io = crash::output;
namespace native = tl::qualification::law44;
std::filesystem::path Readiness;
template<class T> auto Bytes(const T& value) {
    std::array<unsigned char, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    return bytes;
}
class TemporaryDirectory {
  public:
    TemporaryDirectory() {
        const auto pattern = (std::filesystem::temp_directory_path() / "source-material-XXXXXX").string();
        std::vector<char> chars(pattern.begin(), pattern.end());
        chars.push_back(0);
        const auto created = ::mkdtemp(chars.data());
        io::Require(created, "Cannot create source material test directory");
        path = created;
    }
    ~TemporaryDirectory() { std::error_code error; std::filesystem::remove_all(path, error); }
    std::filesystem::path path;
};
TEST(SourcePartMaterial, CompleteOriginalCurveAndRateDeclarationsAreRetained) {
    SourcePartMaterial material;
    ASSERT_FALSE(material.prepared());
    EXPECT_EQ(material.curve().count, 0u);
    const auto loaded = LoadPinnedSourcePartMaterial(Readiness, &material);
    ASSERT_TRUE(loaded) << loaded.message;
    const auto& d = material.declaration();
    EXPECT_EQ(d.part_id, 2000157u);
    EXPECT_EQ(d.material_id, d.part_id);
    EXPECT_EQ(d.section_id, d.part_id);
    EXPECT_EQ(d.curve_id, 2100270u);
    EXPECT_EQ(d.young_pa, 200.e9);
    EXPECT_EQ(d.poisson_ratio, .3);
    EXPECT_EQ(io::Bits(d.density_kg_m3), io::Bits(7889.999999999999));
    EXPECT_EQ(d.thickness_m, .001648);
    EXPECT_EQ(d.supplied_sigy_pa, 270.e6);
    EXPECT_EQ(d.source_rate_coefficient_per_s, 8000.);
    EXPECT_EQ(d.source_rate_exponent, 8.);
    EXPECT_EQ(d.source_vp, 0);
    EXPECT_EQ(d.source_elform, 2u);
    EXPECT_EQ(d.through_thickness_points, 3u);
    const std::array<std::uint16_t, 4> masks{224, 232, 255, 255};
    const std::array<std::uint32_t, 4> lines{3259, 3261, 3263, 3265};
    EXPECT_EQ(d.material_blank_masks, masks);
    EXPECT_EQ(d.material_source_lines, lines);
    EXPECT_TRUE(d.source_lcsr_blank);
    EXPECT_TRUE(d.source_failure_blank);
    EXPECT_TRUE(d.source_tdel_blank);
    EXPECT_EQ(d.filter_cutoff, FilterCutoffResolution::OpenRadiossDirectImportDefault);
    EXPECT_EQ(d.resolved_filter_cutoff_per_s, 10000.);
    const auto curve = material.curve();
    ASSERT_EQ(curve.count, MaterialCurvePoints);
    for (unsigned i = 0; i < MaterialCurvePoints; ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(io::Bits(curve.plastic_strain[i]), io::Bits(native::YarisPlasticStrain[i]));
        EXPECT_EQ(io::Bits(curve.yield_stress_pa[i]), io::Bits(native::YarisYieldStress[i]));
    }
}
TEST(SourcePartMaterial, CopiedAndReloadedMaterialsOwnTheirCurveViews) {
    SourcePartMaterial first;
    ASSERT_TRUE(LoadPinnedSourcePartMaterial(Readiness, &first));
    SourcePartMaterial second = first;
    EXPECT_NE(first.curve().plastic_strain, second.curve().plastic_strain);
    EXPECT_NE(first.curve().yield_stress_pa, second.curve().yield_stress_pa);
    EXPECT_EQ(first.plastic_strain(), second.plastic_strain());
    EXPECT_EQ(first.yield_stress_pa(), second.yield_stress_pa());
    first = SourcePartMaterial{};
    EXPECT_EQ(first.curve().count, 0u);
    ASSERT_EQ(second.curve().count, MaterialCurvePoints);
    EXPECT_EQ(second.curve().yield_stress_pa[45], 362.e6);
    ASSERT_TRUE(LoadPinnedSourcePartMaterial(Readiness, &first));
    EXPECT_EQ(first.plastic_strain(), second.plastic_strain());
    EXPECT_EQ(first.yield_stress_pa(), second.yield_stress_pa());
    tl::material::TabulatedShellPlasticityParameters parameters;
    const auto& d = second.declaration();
    EXPECT_EQ(tl::material::PrepareTabulatedShellPlasticity(
        d.young_pa, d.poisson_ratio, d.density_kg_m3, second.curve(), parameters),
        tl::material::TabulatedShellPlasticityStatus::Ok);
    EXPECT_EQ(parameters.curve.plastic_strain, second.plastic_strain().data());
}
TEST(SourcePartMaterial, RejectedInputPreservesTheEntireLoadedMaterial) {
    SourcePartMaterial material;
    ASSERT_TRUE(LoadPinnedSourcePartMaterial(Readiness, &material));
    const auto held = Bytes(material);
    TemporaryDirectory directory;
    auto bytes = io::ReadBounded(Readiness, 1024 * 1024);
    bytes.back() = bytes.back() == ' ' ? '\n' : ' ';
    io::WriteBytes(directory.path / "changed.json", bytes);
    io::WriteBytes(directory.path / "truncated.json", bytes.substr(0, 100));
    io::WriteBytes(directory.path / "oversized.json", bytes + " ");
    for (const char* name : {"changed.json", "truncated.json"}) {
        EXPECT_EQ(LoadPinnedSourcePartMaterial(directory.path / name, &material).status,
            MaterialStatus::HashMismatch);
        EXPECT_EQ(Bytes(material), held);
    }
    for (const char* name : {"oversized.json", "absent.json"}) {
        EXPECT_EQ(LoadPinnedSourcePartMaterial(directory.path / name, &material).status,
            MaterialStatus::ReadFailure);
        EXPECT_EQ(Bytes(material), held);
    }
    EXPECT_EQ(LoadPinnedSourcePartMaterial(Readiness, nullptr).status, MaterialStatus::InvalidArgument);
    ASSERT_TRUE(LoadPinnedSourcePartMaterial(Readiness, &material));
    EXPECT_EQ(material.plastic_strain(), native::YarisPlasticStrain);
    EXPECT_EQ(material.yield_stress_pa(), native::YarisYieldStress);
}
} // namespace
} // namespace crash::cases::source_part_plastic

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2) return 2;
    crash::cases::source_part_plastic::Readiness = argv[1];
    return RUN_ALL_TESTS();
}
