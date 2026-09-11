// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
extern "C" {
void law90_force_tags(int tags[6]);
void law90_solid_caller_native(const double prepared[33],const double* x,const double* y,
  const int* n,const double base[20],const int cursor[3],const double tensor[6],const double rate[6],
  const double step[5],double values[37],int next_cursor[3],int* status);
void law90_solid_force_native(const double prepared[33],const double* cx,const double* cy,
  const int* n,const double x0[24],const double* rho,const double x[24],const double v[24],
  const double step[2],const double base[160],const int cursors[24],double values[357],
  int next_cursors[24],int* status);
}
