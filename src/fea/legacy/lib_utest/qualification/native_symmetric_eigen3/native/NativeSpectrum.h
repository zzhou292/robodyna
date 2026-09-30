#pragma once
// Input/rate are count contiguous packets of6; output count packets of15:
// eigenvalues3, row-major eigenvector matrix9, native projected rates3.
// Count is passed by reference; 1..129 with the pinned p4linux964 MVSIZ.
extern "C" void spectrum_native(const double*, const double*, const int*, double*, int*);
extern "C" void spectrum_constants(double*);
