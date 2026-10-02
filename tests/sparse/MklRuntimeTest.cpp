#include <gtest/gtest.h>
#include <mkl.h>

#include <array>
#include <filesystem>
#include <link.h>
#include <vector>

namespace {

int FindMklCore(dl_phdr_info* info, size_t, void* opaque) {
    auto& paths = *static_cast<std::vector<std::filesystem::path>*>(opaque);
    std::filesystem::path path(info->dlpi_name);
    if (path.filename() == "libmkl_core.so.2")
        paths.push_back(path);
    return 0;
}

TEST(PardisoSdk, LoadedCoreHasDeclaredDispatchCompanionsAndExecutesBlas) {
    std::vector<std::filesystem::path> cores;
    dl_iterate_phdr(FindMklCore, &cores);
    ASSERT_EQ(cores.size(), 1u) << "Exactly one MKL core must be loaded";
    // Keep the actual loader spelling. Canonicalizing a Bazel _solib symlink
    // here would hide the missing-sibling layout that MKL's dladdr observes.
    const auto directory = cores.front().parent_path();
    const std::array<const char*, 13> companions = {
        "libmkl_avx.so.2", "libmkl_avx2.so.2", "libmkl_avx512.so.2",
        "libmkl_def.so.2", "libmkl_mc.so.2", "libmkl_mc3.so.2",
        "libmkl_vml_avx.so.2", "libmkl_vml_avx2.so.2", "libmkl_vml_avx512.so.2",
        "libmkl_vml_cmpt.so.2", "libmkl_vml_def.so.2", "libmkl_vml_mc.so.2", "libmkl_vml_mc3.so.2",
    };
    for (const auto* name : companions)
        ASSERT_TRUE(std::filesystem::is_regular_file(directory / name)) << directory / name;

    const double a[4] = {1, 2, 3, 4};
    const double b[4] = {2, 0, 1, 3};
    double c[4] = {};
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 2, 2, 2, 1, a, 2, b, 2, 0, c, 2);
    EXPECT_DOUBLE_EQ(c[0], 4);
    EXPECT_DOUBLE_EQ(c[1], 6);
    EXPECT_DOUBLE_EQ(c[2], 10);
    EXPECT_DOUBLE_EQ(c[3], 12);
}

}  // namespace
