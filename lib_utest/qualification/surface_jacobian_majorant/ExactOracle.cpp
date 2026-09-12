// SPDX-License-Identifier: MIT
#include "ExactOracle.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <cmath>
#include <vector>

namespace majorant_test {
namespace {
using Integer = boost::multiprecision::cpp_int;
using Rational = boost::multiprecision::cpp_rational;
// Independent exact conversion of the finite binary64 packet. No decimal text,
// interval helper, floating sqrt or copied production bound is used.
Rational Exact(double value) {
  if (value == 0) return 0;
  int exponent = 0;
  const double fraction = std::frexp(value, &exponent);
  const auto significand = static_cast<std::int64_t>(std::ldexp(fraction, 53));
  Rational result(significand);
  exponent -= 53;
  if (exponent >= 0) result *= Integer(1) << exponent;
  else result /= Integer(1) << -exponent;
  return result;
}
double Component(ct::Vec3 value, unsigned c) {
  return c == 0 ? value.x : c == 1 ? value.y : value.z;
}
bool Form(const ct::SurfaceJacobianMajorant& packet, const std::vector<Rational>& velocities) {
  Rational dot = 0, diagonal = 0;
  for (unsigned i = 0; i < packet.count; ++i) {
    Rational square = 0;
    for (unsigned c = 0; c < 3; ++c) {
      if (packet.nodes[i].translation_fixed_bits & (1u << c)) continue;
      const auto& v = velocities[3 * i + c];
      dot += Exact(Component(packet.nodes[i].jacobian, c)) * v;
      square += v * v;
    }
    diagonal += Exact(packet.nodes[i].diagonal_n_m) * square;
  }
  return Exact(packet.stiffness_n_m) * dot * dot <= diagonal;
}
}
bool ExactBounds(const ct::SurfaceJacobianMajorant& packet) {
  if (!packet.valid || packet.count > ct::MaxSurfaceMajorantNodes) return false;
  Rational sum = 0;
  for (unsigned i = 0; i < packet.count; ++i) {
    const auto& node = packet.nodes[i];
    Rational square = 0;
    for (unsigned c = 0; c < 3; ++c) {
      const Rational j = Exact(Component(node.jacobian, c));
      square += j * j;
      if ((node.translation_fixed_bits & (1u << c)) && j != 0) return false;
    }
    const Rational norm = Exact(node.norm_upper);
    if (norm < 0 || norm * norm < square) return false;
    sum += norm;
    if (Exact(node.diagonal_n_m) < Exact(packet.stiffness_n_m) * norm * Exact(packet.norm_sum_upper)) return false;
  }
  return sum <= Exact(packet.norm_sum_upper);
}
bool ExactQuadratic(const ct::SurfaceJacobianMajorant& packet, const ct::Vec3* velocities) {
  std::vector<Rational> values(3 * packet.count);
  for (unsigned i = 0; i < packet.count; ++i)
    for (unsigned c = 0; c < 3; ++c) values[3 * i + c] = Exact(Component(velocities[i], c));
  return Form(packet, values);
}
bool ExactCongruence(const ct::SurfaceJacobianMajorant& packet, const double* transform,
    unsigned columns, const double* generalized) {
  std::vector<Rational> values(3 * packet.count);
  for (unsigned row = 0; row < 3 * packet.count; ++row)
    for (unsigned column = 0; column < columns; ++column)
      values[row] += Exact(transform[row * columns + column]) * Exact(generalized[column]);
  return Form(packet, values);
}
bool OldWeightBoundFails(double normal) {
  // Signed weights +/-1/2, stiffness 1, trial velocities +/-1. Old D_i=1/2
  // assumes exactly unit n, while actual represented J_x is +/-RN(n/2).
  const Rational j = Exact(.5 * normal);
  return 4 * j * j > Rational(1);
}
} // namespace majorant_test
