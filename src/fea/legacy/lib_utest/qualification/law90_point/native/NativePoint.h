#pragma once
// Prepared33 is the qualified preparation ABI. All arrays use native order.
// History10 and three zero-based cursors are updated by complete SIGEPS90.
// Values11: stress6, SSP, EPSD, ET, VISCMAX, OFF. All scalars passed by reference.
extern "C" void law90_native_point(const double* prepared, const double* x,
    const double* y, const int* count, const double* strain, const double* rate,
    const double* time, double* history, int* cursors, double* values);
