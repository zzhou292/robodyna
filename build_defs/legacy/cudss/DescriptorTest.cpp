#include "lib_src/solvers/CudssApi.h"

#include <gtest/gtest.h>

namespace api = robodyna::fea::cudss_api;

namespace {
struct Matrix {
    cudssMatrix_t value = nullptr;
    ~Matrix() { if (value) cudssMatrixDestroy(value); }
};

TEST(CudssApi, BothRetainedMatrixPoliciesKeepCsr32DoubleUpperZeroBasedStorage) {
    int offsets[] = {0, 2, 3};
    int columns[] = {0, 1, 1};
    double values[] = {4.0, 1.0, 3.0};
    for (const auto policy : {CUDSS_MTYPE_SYMMETRIC, CUDSS_MTYPE_SPD}) {
        Matrix matrix;
        ASSERT_EQ(api::CreateCsr32Double(&matrix.value, 2, 2, 3, offsets,
                                        nullptr, columns, values, policy,
                                        CUDSS_MVIEW_UPPER, CUDSS_BASE_ZERO),
                  CUDSS_STATUS_SUCCESS);
        std::int64_t rows = 0, cols = 0, nonzeros = 0;
        void* row_start = nullptr;
        void* row_end = nullptr;
        void* column_indices = nullptr;
        void* data = nullptr;
        cudssDataType_t offset_type{}, index_type{}, value_type{};
        cudssMatrixType_t type{};
        cudssMatrixViewType_t view{};
        cudssIndexBase_t base{};
        ASSERT_EQ(cudssMatrixGetCsr(matrix.value, &rows, &cols, &nonzeros,
                                   &row_start, &row_end, &column_indices, &data,
                                   &offset_type, &index_type, &value_type,
                                   &type, &view, &base), CUDSS_STATUS_SUCCESS);
        EXPECT_EQ(rows, 2);
        EXPECT_EQ(cols, 2);
        EXPECT_EQ(nonzeros, 3);
        EXPECT_EQ(row_start, offsets);
        EXPECT_EQ(row_end, nullptr);
        EXPECT_EQ(column_indices, columns);
        EXPECT_EQ(data, values);
        EXPECT_EQ(offset_type, CUDSS_R_32I);
        EXPECT_EQ(index_type, CUDSS_R_32I);
        EXPECT_EQ(value_type, CUDSS_R_64F);
        EXPECT_EQ(type, policy);
        EXPECT_EQ(view, CUDSS_MVIEW_UPPER);
        EXPECT_EQ(base, CUDSS_BASE_ZERO);
    }
}

TEST(CudssApi, RightHandSideAndSolutionKeepTheirOwnColumnMajorDoubleStorage) {
    double rhs[] = {1.0, -2.0, 4.0};
    double solution[] = {0.0, 0.0, 0.0};
    for (double* storage : {rhs, solution}) {
        Matrix matrix;
        ASSERT_EQ(api::CreateDenseDouble(&matrix.value, 3, 1, 3, storage,
                                        CUDSS_LAYOUT_COL_MAJOR), CUDSS_STATUS_SUCCESS);
        std::int64_t rows = 0, columns = 0, leading_dimension = 0;
        void* data = nullptr;
        cudssDataType_t type{};
        cudssLayout_t layout{};
        ASSERT_EQ(cudssMatrixGetDn(matrix.value, &rows, &columns, &leading_dimension,
                                  &data, &type, &layout), CUDSS_STATUS_SUCCESS);
        EXPECT_EQ(rows, 3);
        EXPECT_EQ(columns, 1);
        EXPECT_EQ(leading_dimension, 3);
        EXPECT_EQ(data, storage);
        EXPECT_EQ(type, CUDSS_R_64F);
        EXPECT_EQ(layout, CUDSS_LAYOUT_COL_MAJOR);
    }
}

TEST(CudssApi, RenamedDefaultReorderingIsAcceptedAndRetainedByTheHostConfig) {
    struct Config {
        cudssConfig_t value = nullptr;
        ~Config() { if (value) cudssConfigDestroy(value); }
    } config;
    ASSERT_EQ(cudssConfigCreate(&config.value), CUDSS_STATUS_SUCCESS);
    auto requested = api::kDefaultReordering;
    ASSERT_EQ(cudssConfigSet(config.value, CUDSS_CONFIG_REORDERING_ALG,
                            &requested, sizeof(requested)), CUDSS_STATUS_SUCCESS);
    api::ReorderingAlgorithm observed{};
    std::size_t written = 0;
    ASSERT_EQ(cudssConfigGet(config.value, CUDSS_CONFIG_REORDERING_ALG, &observed,
                            sizeof(observed), &written), CUDSS_STATUS_SUCCESS);
    EXPECT_EQ(written, sizeof(observed));
    EXPECT_EQ(observed, CUDSS_REORDERING_ALG_DEFAULT);
}
// These host descriptor/config operations never create a solver handle,
// allocate device memory, or execute analysis, factorization or a GPU kernel.
}  // namespace
