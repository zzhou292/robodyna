#pragma once
#include "lib_src/math/NativeSymmetricEigen3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace spectrum_test {
using Spectrum = tl::math::NativeSymmetricSpectrum3;
using Tensor = std::array<double, 6>;
namespace detail = tl::math::native_symmetric_eigen3;
struct Packet { double tensor[6]{}; double rate[6]{1.3, -2.7, .8, 3.1, -.9, 1.7}; };

TL_NATIVE_SPECTRUM_HD inline void Pack(const Spectrum& spectrum, const double (&rate)[6],
                            double (&values)[15]) noexcept {
  for (unsigned k = 0; k < 3; ++k) values[k] = spectrum.value[k];
  for (unsigned k = 0; k < 9; ++k) values[3+k] = spectrum.vectors.v[k];
  const double ep[6]{rate[0], rate[1], rate[2], .5*rate[3], .5*rate[4], .5*rate[5]};
  const double* v = spectrum.vectors.v;
  for (unsigned k = 0; k < 3; ++k) {
    const double x = v[k]*ep[0] + v[3+k]*ep[3] + v[6+k]*ep[5];
    const double y = v[k]*ep[3] + v[3+k]*ep[1] + v[6+k]*ep[4];
    const double z = v[k]*ep[5] + v[3+k]*ep[4] + v[6+k]*ep[2];
    values[12+k] = v[k]*x + v[3+k]*y + v[6+k]*z;
  }
}
inline bool SameBits(double a, double b) noexcept {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}
inline Packet Diagonal(double a, double b, double c) {
  Packet packet;
  packet.tensor[0] = a;
  packet.tensor[1] = b;
  packet.tensor[2] = c;
  return packet;
}
inline Packet Rotated(double a, double b, double c, double angle, double tilt) {
  const double ca = std::cos(angle), sa = std::sin(angle);
  const double ct = std::cos(tilt), st = std::sin(tilt);
  const double rotation[3][3]{{ca*ct, -sa, ca*st}, {sa*ct, ca, sa*st}, {-st, 0, ct}};
  const double diagonal[3]{a, b, c};
  double matrix[3][3]{};
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      for (unsigned k = 0; k < 3; ++k)
        matrix[i][j] += rotation[i][k] * diagonal[k] * rotation[j][k];
  Packet packet;
  packet.tensor[0] = matrix[0][0];
  packet.tensor[1] = matrix[1][1];
  packet.tensor[2] = matrix[2][2];
  packet.tensor[3] = matrix[0][1];
  packet.tensor[4] = matrix[1][2];
  packet.tensor[5] = matrix[2][0];
  return packet;
}
inline std::vector<Packet> Cases() {
  std::vector<Packet> cases{Diagonal(0, 0, 0), Diagonal(.125, .125, .125),
      Diagonal(-.25, -.25, -.25), Diagonal(.3, .1, -.4),
      Diagonal(-.4, .3, .1), Diagonal(.1, -.4, .3),
      Diagonal(.25, .25, -.5), Diagonal(-.5, .25, .25),
      Diagonal(.25, -.5, .25)};
  for (double gap : {0., 1e-14, 1e-10, 1e-8, 1e-6, .12}) {
    for (double angle : {0., .37, .89, 1.4}) {
      cases.push_back(Rotated(.25+gap, .25-gap, -.5, angle, .61));
      cases.push_back(Rotated(-.25-gap, -.25+gap, .5, angle, -.43));
    }
  }
  for (double scale : {1e-14, 1e-12, 1e-11, 1e-10, 1e-9, 1., 1e6}) {
    cases.push_back(Rotated(.3*scale, .1*scale, -.4*scale, .29, -.52));
  }
  // Both sides and the exact strict near-triple threshold, AAA=norm*1e-10.
  for (double shear : {std::nextafter(1e-10, 0.), 1e-10,
                       std::nextafter(1e-10, 1.)}) {
    Packet packet;
    packet.tensor[3] = shear;
    cases.push_back(packet);
  }
  for (unsigned n = 0; n < 129; ++n) {
    const double t = static_cast<double>(n) / 128;
    cases.push_back(Rotated(.21+.2*t, .03-.12*t, -.31-.08*t, .05+2.3*t, -.9+1.4*t));
  }
  return cases;
}
inline double TensorScale(const Packet& packet) {
  double scale = 1e-20;
  for (double value : packet.tensor) scale = std::max(scale, std::abs(value));
  return scale;
}
inline bool Agree(const double (&actual)[15], const double* expected,
                  const Packet& packet, double relative = 2e-12) {
  for (unsigned k = 0; k < 15; ++k) {
    const double scale = k < 3 ? TensorScale(packet) : k < 12 ? 1. : 10.;
    if (!std::isfinite(actual[k]) || !std::isfinite(expected[k]) ||
        std::abs(actual[k]-expected[k]) > relative*scale) return false;
  }
  return true;
}
inline double Residual(const Packet& packet, const Spectrum& spectrum) {
  const double matrix[3][3]{{packet.tensor[0], packet.tensor[3], packet.tensor[5]},
      {packet.tensor[3], packet.tensor[1], packet.tensor[4]},
      {packet.tensor[5], packet.tensor[4], packet.tensor[2]}};
  double error = 0;
  for (unsigned r = 0; r < 3; ++r) {
    for (unsigned c = 0; c < 3; ++c) {
      long double reconstructed = 0;
      for (unsigned k = 0; k < 3; ++k)
        reconstructed += static_cast<long double>(spectrum.vectors.v[3*r+k]) *
            spectrum.value[k] * spectrum.vectors.v[3*c+k];
      error = std::max(error, static_cast<double>(std::abs(reconstructed-matrix[r][c])));
    }
  }
  return error;
}
} // namespace spectrum_test
