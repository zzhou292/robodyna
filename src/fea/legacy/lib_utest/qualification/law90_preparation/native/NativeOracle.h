#pragma once
extern "C" {
void law90_native_prepare(const double input[13], const int flags[3],
                          const double* x, const double* y, const int* count,
                          double values[33]);
void law90_native_curve(const double* x, const double* y, const int* count,
                        const double* query, const int* cursor,
                        double values[2], int* next);
}
