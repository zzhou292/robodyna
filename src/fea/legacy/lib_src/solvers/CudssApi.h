#pragma once

#include <cstdint>
#include <cudss.h>

// cuDSS 0.8 split CSR offset/index types and renamed its reordering enum.
// This adapter preserves the retained Newton solver's 32-bit CSR and double
// storage. It changes descriptor API spelling only, not factorization policy.
// https://docs.nvidia.com/cuda/cudss/migration_guide.html
#if CUDSS_VERSION != 800
#error "Review the cuDSS descriptor adapter before changing the qualified 0.8.0 SDK"
#endif

namespace robodyna::fea::cudss_api {

static_assert(sizeof(int) == 4, "The retained CSR arrays use 32-bit int storage");
static_assert(static_cast<int>(CUDSS_R_32I) == static_cast<int>(CUDA_R_32I));
static_assert(static_cast<int>(CUDSS_R_64F) == static_cast<int>(CUDA_R_64F));

using ReorderingAlgorithm = cudssReorderingAlg_t;
inline constexpr ReorderingAlgorithm kDefaultReordering = CUDSS_REORDERING_ALG_DEFAULT;

inline cudssStatus_t CreateCsr32Double(cudssMatrix_t* matrix,
                                     std::int64_t rows, std::int64_t columns,
                                     std::int64_t nonzeros, const int* row_start,
                                     const int* row_end, const int* column_indices,
                                     const double* values, cudssMatrixType_t type,
                                     cudssMatrixViewType_t view, cudssIndexBase_t base) {
    return cudssMatrixCreateCsr(matrix, rows, columns, nonzeros, row_start, row_end,
                               column_indices, values, CUDSS_R_32I, CUDSS_R_32I,
                               CUDSS_R_64F, type, view, base);
}

inline cudssStatus_t CreateDenseDouble(cudssMatrix_t* matrix,
                                     std::int64_t rows, std::int64_t columns,
                                     std::int64_t leading_dimension,
                                     const double* values, cudssLayout_t layout) {
    return cudssMatrixCreateDn(matrix, rows, columns, leading_dimension, values,
                              CUDSS_R_64F, layout);
}

}  // namespace robodyna::fea::cudss_api
