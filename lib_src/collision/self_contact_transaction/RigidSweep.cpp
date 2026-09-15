// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace tlfea::contact::self_contact_transaction {
namespace {

struct Interval {
  double lower = 0;
  double upper = 0;
};

// Every finite binary64 value is an integer multiple of 2^-1074.  A weighted
// rigid-member curvature contains four input factors (weight, omega, omega,
// arm), so 144 fixed limbs are sufficient for the exact represented sum:
// 4*2098 bits plus signs, additions, and carry.  This certificate allocates no
// memory and returns unsupported if the stated arithmetic envelope is ever
// exceeded.
constexpr unsigned ExactLimbs = 144;

struct ExactInteger {
  std::uint64_t limbs[ExactLimbs]{};
  unsigned used = 0;
  bool negative = false;
  bool overflow = false;
};

void Normalize(ExactInteger* value) noexcept {
  while (value->used && !value->limbs[value->used - 1])
    --value->used;
  if (!value->used) value->negative = false;
}

ExactInteger Exact(double value) noexcept {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  const unsigned raw =
      static_cast<unsigned>((bits >> 52) & 0x7ffu);
  const std::uint64_t fraction =
      bits & ((std::uint64_t{1} << 52) - 1);
  ExactInteger result;
  const std::uint64_t significand =
      raw ? (std::uint64_t{1} << 52) | fraction : fraction;
  if (!significand) return result;
  const int exponent =
      raw ? static_cast<int>(raw) - 1023 - 52 : -1074;
  const unsigned shift = static_cast<unsigned>(exponent + 1074);
  const unsigned word = shift / 64;
  const unsigned bit = shift % 64;
  result.limbs[word] = significand << bit;
  result.used = word + 1;
  if (bit && significand >> (64 - bit)) {
    result.limbs[word + 1] = significand >> (64 - bit);
    result.used = word + 2;
  }
  result.negative = (bits >> 63) != 0;
  return result;
}

int CompareMagnitude(
    const ExactInteger& first, const ExactInteger& second) noexcept {
  if (first.used != second.used)
    return first.used < second.used ? -1 : 1;
  for (unsigned i = first.used; i; --i)
    if (first.limbs[i - 1] != second.limbs[i - 1])
      return first.limbs[i - 1] < second.limbs[i - 1] ? -1 : 1;
  return 0;
}

ExactInteger AddMagnitude(
    const ExactInteger& first, const ExactInteger& second) noexcept {
  ExactInteger result;
  result.overflow = first.overflow || second.overflow;
  const unsigned count = std::max(first.used, second.used);
  unsigned __int128 carry = 0;
  for (unsigned i = 0; i < count; ++i) {
    const unsigned __int128 value =
        static_cast<unsigned __int128>(
            i < first.used ? first.limbs[i] : 0) +
        (i < second.used ? second.limbs[i] : 0) + carry;
    result.limbs[i] = static_cast<std::uint64_t>(value);
    carry = value >> 64;
  }
  result.used = count;
  if (carry) {
    if (count == ExactLimbs) result.overflow = true;
    else {
      result.limbs[count] = static_cast<std::uint64_t>(carry);
      result.used = count + 1;
    }
  }
  return result;
}

ExactInteger SubtractMagnitude(
    const ExactInteger& larger, const ExactInteger& smaller) noexcept {
  ExactInteger result;
  result.overflow = larger.overflow || smaller.overflow;
  std::uint64_t borrow = 0;
  for (unsigned i = 0; i < larger.used; ++i) {
    const std::uint64_t right =
        i < smaller.used ? smaller.limbs[i] : 0;
    const std::uint64_t first = larger.limbs[i] - right;
    const std::uint64_t borrow_first = larger.limbs[i] < right;
    const std::uint64_t second = first - borrow;
    const std::uint64_t borrow_second = first < borrow;
    result.limbs[i] = second;
    borrow = borrow_first | borrow_second;
  }
  result.used = larger.used;
  result.overflow = result.overflow || borrow;
  Normalize(&result);
  return result;
}

ExactInteger Add(ExactInteger first, ExactInteger second) noexcept {
  if (first.negative == second.negative) {
    auto result = AddMagnitude(first, second);
    result.negative = first.negative && result.used;
    return result;
  }
  const int order = CompareMagnitude(first, second);
  if (!order) {
    ExactInteger result;
    result.overflow = first.overflow || second.overflow;
    return result;
  }
  const bool first_larger = order > 0;
  auto result = SubtractMagnitude(
      first_larger ? first : second, first_larger ? second : first);
  result.negative =
      (first_larger ? first.negative : second.negative) && result.used;
  return result;
}

ExactInteger Negate(ExactInteger value) noexcept {
  if (value.used) value.negative = !value.negative;
  return value;
}

ExactInteger Subtract(
    ExactInteger first, ExactInteger second) noexcept {
  return Add(first, Negate(second));
}

ExactInteger Multiply(
    const ExactInteger& first, const ExactInteger& second) noexcept {
  ExactInteger result;
  result.overflow = first.overflow || second.overflow;
  if (!first.used || !second.used) return result;
  for (unsigned i = 0; i < first.used; ++i) {
    unsigned __int128 carry = 0;
    for (unsigned j = 0; j < second.used; ++j) {
      if (i + j >= ExactLimbs) {
        result.overflow = true;
        continue;
      }
      const unsigned __int128 value =
          static_cast<unsigned __int128>(
              first.limbs[i]) * second.limbs[j] +
          result.limbs[i + j] + carry;
      result.limbs[i + j] = static_cast<std::uint64_t>(value);
      carry = value >> 64;
    }
    unsigned at = i + second.used;
    while (carry) {
      if (at >= ExactLimbs) {
        result.overflow = true;
        break;
      }
      const unsigned __int128 value =
          static_cast<unsigned __int128>(result.limbs[at]) + carry;
      result.limbs[at] = static_cast<std::uint64_t>(value);
      carry = value >> 64;
      ++at;
    }
  }
  result.used =
      std::min<unsigned>(ExactLimbs, first.used + second.used);
  result.negative = first.negative != second.negative;
  Normalize(&result);
  return result;
}

bool ExactCross(
    const ExactInteger first[3], const ExactInteger second[3],
    ExactInteger output[3]) noexcept {
  for (unsigned component = 0; component < 3; ++component) {
    const unsigned a = (component + 1) % 3;
    const unsigned b = (component + 2) % 3;
    output[component] = Subtract(
        Multiply(first[a], second[b]),
        Multiply(first[b], second[a]));
    if (output[component].overflow) return false;
  }
  return true;
}

unsigned HighestBit(const ExactInteger& value) noexcept {
  const auto top = value.limbs[value.used - 1];
  return 64 * (value.used - 1) +
      63u - static_cast<unsigned>(__builtin_clzll(top));
}

std::uint64_t ShiftedLow(
    const ExactInteger& value, unsigned shift) noexcept {
  const unsigned word = shift / 64;
  const unsigned bit = shift % 64;
  if (word >= value.used) return 0;
  std::uint64_t result = value.limbs[word] >> bit;
  if (bit && word + 1 < value.used)
    result |= value.limbs[word + 1] << (64 - bit);
  return result;
}

bool HasLowBits(
    const ExactInteger& value, unsigned count) noexcept {
  const unsigned words = count / 64;
  const unsigned bits = count % 64;
  for (unsigned i = 0; i < std::min(words, value.used); ++i)
    if (value.limbs[i]) return true;
  return bits && words < value.used &&
      (value.limbs[words] &
       ((std::uint64_t{1} << bits) - 1));
}

double FromBits(std::uint64_t bits) noexcept {
  double result = 0;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

bool ExactCoefficientBounds(
    const ExactInteger& value, DirectedInterval* output) noexcept {
  if (!output || value.overflow) return false;
  if (!value.used) {
    *output = {};
    return true;
  }
  // represented_q has four factors, each aligned to 2^-1074.
  constexpr int ScaleExponent = -4296;
  const unsigned highest = HighestBit(value);
  const int exponent = static_cast<int>(highest) + ScaleExponent;
  if (exponent > 1023) return false;
  double magnitude_lower = 0;
  double magnitude_upper = 0;
  if (exponent >= -1022) {
    const unsigned shift = highest > 52 ? highest - 52 : 0;
    std::uint64_t significand = ShiftedLow(value, shift);
    if (highest < 52) significand <<= 52 - highest;
    const std::uint64_t raw_exponent =
        static_cast<std::uint64_t>(exponent + 1023);
    const std::uint64_t bits =
        (raw_exponent << 52) |
        (significand - (std::uint64_t{1} << 52));
    magnitude_lower = FromBits(bits);
    magnitude_upper = HasLowBits(value, shift)
        ? std::nextafter(
              magnitude_lower,
              std::numeric_limits<double>::infinity())
        : magnitude_lower;
  } else {
    constexpr unsigned SubnormalShift = 3222;
    const std::uint64_t units =
        ShiftedLow(value, SubnormalShift);
    if (units > (std::uint64_t{1} << 52)) return false;
    magnitude_lower = FromBits(units);
    magnitude_upper = HasLowBits(value, SubnormalShift)
        ? std::nextafter(
              magnitude_lower,
              std::numeric_limits<double>::infinity())
        : magnitude_lower;
  }
  if (!std::isfinite(magnitude_lower) ||
      !std::isfinite(magnitude_upper))
    return false;
  *output = value.negative
      ? DirectedInterval{-magnitude_upper, -magnitude_lower}
      : DirectedInterval{magnitude_lower, magnitude_upper};
  return true;
}

double Down(double value) noexcept {
  return std::nextafter(
      value, -std::numeric_limits<double>::infinity());
}

double Up(double value) noexcept {
  return std::nextafter(
      value, std::numeric_limits<double>::infinity());
}

double Component(Vec3 value, unsigned component) noexcept {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

double Component(tl::math::Vec3 value,
                 unsigned component) noexcept {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

void SetComponent(Vec3* value, unsigned component,
                  double next) noexcept {
  if (component == 0) value->x = next;
  else if (component == 1) value->y = next;
  else value->z = next;
}

bool Difference(double first, double second,
                Interval* output) noexcept {
  if (!output || !std::isfinite(first) ||
      !std::isfinite(second))
    return false;
  if (first == second) {
    *output = {};
    return true;
  }
  const double value = first - second;
  if (!std::isfinite(value)) return false;
  *output = {Down(value), Up(value)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool Product(const Interval& first, const Interval& second,
             Interval* output) noexcept {
  if (!output ||
      !std::isfinite(first.lower) ||
      !std::isfinite(first.upper) ||
      !std::isfinite(second.lower) ||
      !std::isfinite(second.upper) ||
      first.lower > first.upper ||
      second.lower > second.upper)
    return false;
  if ((first.lower == 0 && first.upper == 0) ||
      (second.lower == 0 && second.upper == 0)) {
    *output = {};
    return true;
  }
  const double products[4]{
      first.lower * second.lower,
      first.lower * second.upper,
      first.upper * second.lower,
      first.upper * second.upper};
  double lower = products[0], upper = products[0];
  for (const double value : products) {
    if (!std::isfinite(value)) return false;
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  *output = {Down(lower), Up(upper)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool Subtract(const Interval& first, const Interval& second,
              Interval* output) noexcept {
  if (!output) return false;
  const double lower = first.lower - second.upper;
  const double upper = first.upper - second.lower;
  if (!std::isfinite(lower) || !std::isfinite(upper))
    return false;
  *output = {Down(lower), Up(upper)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool Cross(const Interval first[3], const Interval second[3],
           Interval output[3]) noexcept {
  for (unsigned component = 0; component < 3; ++component) {
    const unsigned a = (component + 1) % 3;
    const unsigned b = (component + 2) % 3;
    Interval positive, negative;
    if (!Product(first[a], second[b], &positive) ||
        !Product(first[b], second[a], &negative) ||
        !Subtract(positive, negative, output + component))
      return false;
  }
  return true;
}

bool SameIdentity(
    const tl::fea::NodalRigidGroupSnapshot& first,
    const tl::fea::NodalRigidGroupSnapshot& second) noexcept {
  return first.source_kind == second.source_kind &&
      first.source_group_id == second.source_group_id &&
      first.source_node_set_id == second.source_node_set_id;
}

RigidMemberSweepStatus ValidateRigidPolynomial(
    Vec3 accepted_member, Vec3 prepared_member,
    const tl::fea::NodalRigidGroupSnapshot& accepted_group,
    const tl::fea::NodalRigidGroupSnapshot& prepared_group,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration) noexcept {
  using R = RigidMemberSweepStatus;
  if (!SameIdentity(accepted_group, prepared_group) ||
      trajectory != tl::fea::NodalRigidMemberTrajectory::
          EndpointCorrectedSecondOrderDriftV1 ||
      !(duration > 0) || !std::isfinite(duration) ||
      !IsFinite(accepted_member) || !IsFinite(prepared_member) ||
      !tl::fea::rigid::detail::Finite(accepted_group.state.center) ||
      !tl::fea::rigid::detail::Finite(prepared_group.state.center) ||
      !tl::fea::rigid::detail::Finite(prepared_group.state.omega))
    return trajectory ==
            tl::fea::NodalRigidMemberTrajectory::
                EndpointCorrectedSecondOrderDriftV1
        ? R::InvalidInput : R::UnsupportedTrajectory;
  const auto omega = prepared_group.state.omega;
  constexpr long double Pi =
      3.141592653589793238462643383279502884L;
  const long double wx = static_cast<long double>(omega.x);
  const long double wy = static_cast<long double>(omega.y);
  const long double wz = static_cast<long double>(omega.z);
  const long double angle =
      static_cast<long double>(duration) *
      std::sqrt(wx * wx + wy * wy + wz * wz);
  if (!std::isfinite(angle)) return R::NonfiniteResult;
  return angle < Pi ? R::Ok : R::RotationLimit;
}

}  // namespace

RigidMemberSweepStatus BuildRigidPointQuadraticCoefficients(
    const WeightedSurfacePoint& point,
    VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, DirectedInterval output[3],
    bool* affine) noexcept {
  using R = RigidMemberSweepStatus;
  if (!output || !affine || !node_rigid_groups ||
      !accepted.valid() || !prepared.valid() ||
      ValidateWeightedSurfacePoint(
          point, accepted.node_count) != Status::kOk ||
      ValidateWeightedSurfacePoint(
          point, prepared.node_count) != Status::kOk)
    return R::InvalidInput;
  *affine = false;
  ExactInteger represented_q[3];
  for (unsigned slot = 0; slot < point.count; ++slot) {
    const auto node = point.nodes[slot];
    const double weight = point.weights[slot];
    const auto accepted_member = accepted.at(node);
    const auto prepared_member = prepared.at(node);
    if (!IsFinite(accepted_member) || !IsFinite(prepared_member))
      return R::InvalidInput;
    const auto group = node_rigid_groups[node];
    if (weight == 0 || group == UINT32_MAX) continue;
    if (group >= group_count || !accepted_groups || !prepared_groups)
      return R::InvalidInput;
    const auto status = ValidateRigidPolynomial(
        accepted_member, prepared_member,
        accepted_groups[group], prepared_groups[group],
        trajectory, duration);
    if (status != R::Ok) return status;
    const auto spin = prepared_groups[group].state.omega;
    if (spin.x == 0 && spin.y == 0 && spin.z == 0)
      continue;

    ExactInteger arm[3], omega[3], tangent[3], q[3];
    for (unsigned component = 0; component < 3; ++component) {
      arm[component] = Subtract(
          Exact(Component(accepted_member, component)),
          Exact(Component(
              accepted_groups[group].state.center, component)));
      omega[component] = Exact(Component(
          prepared_groups[group].state.omega, component));
    }
    if (!ExactCross(omega, arm, tangent) ||
        !ExactCross(omega, tangent, q))
      return R::NonfiniteResult;
    const auto exact_weight = Exact(weight);
    for (unsigned component = 0; component < 3; ++component) {
      represented_q[component] = Add(
          represented_q[component],
          Multiply(exact_weight, q[component]));
      if (represented_q[component].overflow)
        return R::NonfiniteResult;
    }
  }
  *affine = !represented_q[0].used &&
      !represented_q[1].used && !represented_q[2].used;
  for (unsigned component = 0; component < 3; ++component)
    if (!ExactCoefficientBounds(
            represented_q[component], output + component))
      return R::NonfiniteResult;
  return R::Ok;
}

RigidMemberSweepStatus CertifyRigidPointAffineMotion(
    const WeightedSurfacePoint& point,
    VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, bool* affine) noexcept {
  DirectedInterval q[3];
  return BuildRigidPointQuadraticCoefficients(
      point, accepted, prepared, node_rigid_groups,
      accepted_groups, prepared_groups, group_count,
      trajectory, duration, q, affine);
}

RigidMemberSweepStatus CertifyRigidFacetAffineMotion(
    const FixedContactFacet& facet,
    VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, bool* affine) noexcept {
  FacetQuadraticCoefficients coefficients;
  return BuildRigidFacetQuadraticCoefficients(
      facet, accepted, prepared, node_rigid_groups,
      accepted_groups, prepared_groups, group_count,
      trajectory, duration, &coefficients, affine);
}

RigidMemberSweepStatus BuildRigidFacetQuadraticCoefficients(
    const FixedContactFacet& facet,
    VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, FacetQuadraticCoefficients* output,
    bool* affine) noexcept {
  using R = RigidMemberSweepStatus;
  if (!output || !affine) return R::InvalidInput;
  *output = {};
  *affine = false;
  bool all_affine = true;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    bool point_affine = false;
    const auto status = BuildRigidPointQuadraticCoefficients(
        facet.vertices[vertex], accepted, prepared,
        node_rigid_groups, accepted_groups, prepared_groups,
        group_count, trajectory, duration, output->q[vertex],
        &point_affine);
    if (status != R::Ok) return status;
    all_affine = all_affine && point_affine;
  }
  output->complete = true;
  *affine = all_affine;
  return R::Ok;
}

namespace {

struct BernsteinCoordinate {
  DirectedInterval control[3];
};

struct BernsteinFacet {
  BernsteinCoordinate coordinate[3][3];
};

bool Valid(DirectedInterval value) noexcept {
  return std::isfinite(value.lower) &&
      std::isfinite(value.upper) &&
      value.lower <= value.upper;
}

bool MultiplyDirected(
    DirectedInterval first, DirectedInterval second,
    DirectedInterval* output) noexcept {
  if (!output || !Valid(first) || !Valid(second)) return false;
  if ((first.lower == 0 && first.upper == 0) ||
      (second.lower == 0 && second.upper == 0)) {
    *output = {};
    return true;
  }
  const double values[4]{
      first.lower * second.lower,
      first.lower * second.upper,
      first.upper * second.lower,
      first.upper * second.upper};
  double lower = values[0], upper = values[0];
  for (const auto value : values) {
    if (!std::isfinite(value)) return false;
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  *output = {Down(lower), Up(upper)};
  return Valid(*output);
}

bool AverageDirected(
    DirectedInterval first, DirectedInterval second,
    DirectedInterval* output) noexcept {
  if (!output || !Valid(first) || !Valid(second)) return false;
  const double first_lower = Down(.5 * first.lower);
  const double second_lower = Down(.5 * second.lower);
  const double first_upper = Up(.5 * first.upper);
  const double second_upper = Up(.5 * second.upper);
  *output = {
      Down(first_lower + second_lower),
      Up(first_upper + second_upper)};
  return Valid(*output);
}

bool SubtractDirected(
    DirectedInterval first, DirectedInterval second,
    DirectedInterval* output) noexcept {
  if (!output || !Valid(first) || !Valid(second)) return false;
  *output = {
      Down(first.lower - second.upper),
      Up(first.upper - second.lower)};
  return Valid(*output);
}

bool BuildBernsteinFacet(
    const CurrentFixedTriangle& accepted,
    const CurrentFixedTriangle& prepared,
    const FacetQuadraticCoefficients& coefficients,
    double duration, BernsteinFacet* output) noexcept {
  if (!output || !coefficients.complete ||
      !(duration > 0) || !std::isfinite(duration))
    return false;
  const double h2 = duration * duration;
  if (!std::isfinite(h2)) return false;
  const DirectedInterval h2_interval{Down(h2), Up(h2)};
  const DirectedInterval quarter{.25, .25};
  BernsteinFacet next;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    if (!IsFinite(accepted.vertices[vertex]) ||
        !IsFinite(prepared.vertices[vertex]))
      return false;
    for (unsigned component = 0; component < 3; ++component) {
      auto& control = next.coordinate[vertex][component].control;
      control[0] = {
          Component(accepted.vertices[vertex], component),
          Component(accepted.vertices[vertex], component)};
      control[2] = {
          Component(prepared.vertices[vertex], component),
          Component(prepared.vertices[vertex], component)};
      DirectedInterval endpoint_midpoint, scaled_q, quarter_q;
      if (!Valid(coefficients.q[vertex][component]) ||
          !AverageDirected(control[0], control[2],
                           &endpoint_midpoint) ||
          !MultiplyDirected(
              coefficients.q[vertex][component],
              h2_interval, &scaled_q) ||
          !MultiplyDirected(scaled_q, quarter, &quarter_q) ||
          !SubtractDirected(
              endpoint_midpoint, quarter_q, control + 1))
        return false;
    }
  }
  *output = next;
  return true;
}

bool SplitCoordinate(
    const BernsteinCoordinate& input,
    BernsteinCoordinate* left,
    BernsteinCoordinate* right) noexcept {
  DirectedInterval first, second, midpoint;
  if (!left || !right ||
      !AverageDirected(input.control[0], input.control[1], &first) ||
      !AverageDirected(input.control[1], input.control[2], &second) ||
      !AverageDirected(first, second, &midpoint))
    return false;
  left->control[0] = input.control[0];
  left->control[1] = first;
  left->control[2] = midpoint;
  right->control[0] = midpoint;
  right->control[1] = second;
  right->control[2] = input.control[2];
  return true;
}

bool SplitFacet(
    const BernsteinFacet& input,
    BernsteinFacet* left, BernsteinFacet* right) noexcept {
  if (!left || !right) return false;
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    for (unsigned component = 0; component < 3; ++component)
      if (!SplitCoordinate(
              input.coordinate[vertex][component],
              &left->coordinate[vertex][component],
              &right->coordinate[vertex][component]))
        return false;
  return true;
}

bool FacetBounds(
    const BernsteinFacet& facet, unsigned component,
    double thickness, DirectedInterval* output) noexcept {
  if (!output || !(thickness > 0) || !std::isfinite(thickness))
    return false;
  double lower =
      facet.coordinate[0][component].control[0].lower;
  double upper =
      facet.coordinate[0][component].control[0].upper;
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    for (unsigned control = 0; control < 3; ++control) {
      const auto value =
          facet.coordinate[vertex][component].control[control];
      if (!Valid(value)) return false;
      lower = std::min(lower, value.lower);
      upper = std::max(upper, value.upper);
    }
  *output = {Down(lower - thickness), Up(upper + thickness)};
  return Valid(*output);
}

Vec3 Difference(Vec3 first, Vec3 second) noexcept {
  return {
      first.x - second.x,
      first.y - second.y,
      first.z - second.z};
}

Vec3 CrossAxis(Vec3 first, Vec3 second) noexcept {
  return {
      first.y * second.z - first.z * second.y,
      first.z * second.x - first.x * second.z,
      first.x * second.y - first.y * second.x};
}

Vec3 EdgeAxis(Vec3 first, Vec3 second) noexcept {
  return Difference(second, first);
}

bool Representative(
    const BernsteinFacet& facet, Vec3 output[3]) noexcept {
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    for (unsigned component = 0; component < 3; ++component) {
      const auto value =
          facet.coordinate[vertex][component].control[0];
      if (!Valid(value)) return false;
      const double representative =
          .5 * value.lower + .5 * value.upper;
      if (!std::isfinite(representative)) return false;
      SetComponent(output + vertex, component, representative);
    }
  return true;
}

bool ProjectionBounds(
    const BernsteinFacet& facet, Vec3 axis,
    DirectedInterval* output) noexcept {
  if (!output || !IsFinite(axis)) return false;
  bool first_value = true;
  DirectedInterval bounds;
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    for (unsigned control = 0; control < 3; ++control) {
      DirectedInterval projection{};
      for (unsigned component = 0; component < 3; ++component) {
        DirectedInterval product;
        const double axis_component = Component(axis, component);
        if (!MultiplyDirected(
                facet.coordinate[vertex][component].control[control],
                {axis_component, axis_component}, &product))
          return false;
        if (component == 0) {
          projection = product;
        } else {
          projection = {
              Down(projection.lower + product.lower),
              Up(projection.upper + product.upper)};
          if (!Valid(projection)) return false;
        }
      }
      if (first_value) {
        bounds = projection;
        first_value = false;
      } else {
        bounds.lower = std::min(bounds.lower, projection.lower);
        bounds.upper = std::max(bounds.upper, projection.upper);
      }
    }
  *output = bounds;
  return !first_value && Valid(*output);
}

bool AxisSeparates(
    const BernsteinFacet& first, double first_thickness,
    const BernsteinFacet& second, double second_thickness,
    Vec3 axis, bool* valid) noexcept {
  if (!valid) return false;
  *valid = false;
  if (!IsFinite(axis)) return false;
  const double norm_l1 = Up(
      Up(std::fabs(axis.x) + std::fabs(axis.y)) +
      std::fabs(axis.z));
  if (!std::isfinite(norm_l1)) return false;
  if (!(norm_l1 > 0)) {
    *valid = true;
    return false;
  }
  DirectedInterval first_projection, second_projection;
  if (!ProjectionBounds(first, axis, &first_projection) ||
      !ProjectionBounds(second, axis, &second_projection))
    return false;
  const double first_margin = Up(first_thickness * norm_l1);
  const double second_margin = Up(second_thickness * norm_l1);
  if (!std::isfinite(first_margin) ||
      !std::isfinite(second_margin))
    return false;
  first_projection = {
      Down(first_projection.lower - first_margin),
      Up(first_projection.upper + first_margin)};
  second_projection = {
      Down(second_projection.lower - second_margin),
      Up(second_projection.upper + second_margin)};
  if (!Valid(first_projection) || !Valid(second_projection))
    return false;
  *valid = true;
  return first_projection.upper < second_projection.lower ||
      second_projection.upper < first_projection.lower;
}

bool CoordinateSeparated(
    const BernsteinFacet& first, double first_thickness,
    const BernsteinFacet& second, double second_thickness,
    bool* valid) noexcept {
  if (!valid) return false;
  *valid = false;
  for (unsigned component = 0; component < 3; ++component) {
    DirectedInterval a, b;
    if (!FacetBounds(first, component, first_thickness, &a) ||
        !FacetBounds(second, component, second_thickness, &b))
      return false;
    if (a.upper < b.lower || b.upper < a.lower) {
      *valid = true;
      return true;
    }
  }
  Vec3 representative[2][3];
  if (!Representative(first, representative[0]) ||
      !Representative(second, representative[1]))
    return false;
  *valid = true;
  const auto test_axis = [&](Vec3 axis) noexcept {
    bool axis_valid = false;
    const bool separated = AxisSeparates(
        first, first_thickness, second, second_thickness,
        axis, &axis_valid);
    if (!axis_valid) *valid = false;
    return separated;
  };
  for (unsigned side = 0; side < 2; ++side) {
    const auto edge0 = EdgeAxis(
        representative[side][0], representative[side][1]);
    const auto edge1 = EdgeAxis(
        representative[side][0], representative[side][2]);
    if (test_axis(CrossAxis(edge0, edge1))) return true;
    if (!*valid) return false;
  }
  for (unsigned first_edge = 0; first_edge < 3; ++first_edge) {
    const auto a = EdgeAxis(
        representative[0][first_edge],
        representative[0][(first_edge + 1) % 3]);
    for (unsigned second_edge = 0; second_edge < 3; ++second_edge) {
      const auto b = EdgeAxis(
          representative[1][second_edge],
          representative[1][(second_edge + 1) % 3]);
      if (test_axis(CrossAxis(a, b))) return true;
      if (!*valid) return false;
    }
  }
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    for (unsigned edge = 0; edge < 3; ++edge) {
      for (unsigned order = 0; order < 2; ++order) {
        const auto& vertex_triangle = representative[order];
        const auto& edge_triangle = representative[1 - order];
        const auto edge_axis = EdgeAxis(
            edge_triangle[edge], edge_triangle[(edge + 1) % 3]);
        const auto from_start = Difference(
            vertex_triangle[vertex], edge_triangle[edge]);
        if (test_axis(CrossAxis(
                edge_axis, CrossAxis(from_start, edge_axis))))
          return true;
        if (!*valid) return false;
      }
    }
  }
  for (unsigned first_vertex = 0; first_vertex < 3; ++first_vertex)
    for (unsigned second_vertex = 0; second_vertex < 3; ++second_vertex) {
      if (test_axis(Difference(
              representative[1][second_vertex],
              representative[0][first_vertex])))
        return true;
      if (!*valid) return false;
    }
  *valid = true;
  return false;
}

NonlinearSeparationStatus SubdivideSeparation(
    const BernsteinFacet& first,
    const BernsteinFacet& second,
    double first_thickness, double second_thickness,
    unsigned depth, unsigned max_depth,
    std::size_t max_work, std::size_t* work,
    unsigned* deepest) noexcept {
  // Proof.  On one time interval, each vertex coordinate is a degree-two
  // Bernstein polynomial whose exact-real controls lie in the stored directed
  // intervals.  The Bernstein convex-hull property therefore puts every
  // coordinate, and every fixed-axis projection, inside the hull tested by
  // CoordinateSeparated.  Spherical half-thickness projects by at most
  // thickness*|axis|_2 <= thickness*|axis|_1, so a strict projected interval
  // gap separates both thickened facets for every u in this interval.
  //
  // Directed de Casteljau averages at 1/2 enclose the exact controls of the
  // two dyadic children.  Thus a node proves its whole interval directly, or
  // proves it inductively only after both synchronous children do.  Any
  // arithmetic, work, or depth failure returns unresolved and can never
  // become a separation certificate.
  if (!work || !deepest || *work >= max_work)
    return NonlinearSeparationStatus::WorkExhausted;
  ++*work;
  *deepest = std::max(*deepest, depth);
  bool valid = false;
  if (CoordinateSeparated(
          first, first_thickness, second, second_thickness, &valid))
    return valid ? NonlinearSeparationStatus::CertifiedSeparated
                 : NonlinearSeparationStatus::InvalidInput;
  if (!valid) return NonlinearSeparationStatus::InvalidInput;
  if (depth >= max_depth)
    return NonlinearSeparationStatus::DepthExhausted;
  BernsteinFacet first_left, first_right, second_left, second_right;
  if (!SplitFacet(first, &first_left, &first_right) ||
      !SplitFacet(second, &second_left, &second_right))
    return NonlinearSeparationStatus::InvalidInput;
  const auto left = SubdivideSeparation(
      first_left, second_left, first_thickness, second_thickness,
      depth + 1, max_depth, max_work, work, deepest);
  if (left != NonlinearSeparationStatus::CertifiedSeparated)
    return left;
  return SubdivideSeparation(
      first_right, second_right, first_thickness, second_thickness,
      depth + 1, max_depth, max_work, work, deepest);
}

}  // namespace

NonlinearSeparationResult CertifyQuadraticFacetSeparation(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_coefficients,
    double first_thickness,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_coefficients,
    double second_thickness, double duration,
    std::size_t max_work, unsigned max_depth) noexcept {
  NonlinearSeparationResult result;
  if (!max_work || max_depth > 52 ||
      !(first_thickness > 0) ||
      !std::isfinite(first_thickness) ||
      !(second_thickness > 0) ||
      !std::isfinite(second_thickness))
    return result;
  BernsteinFacet first, second;
  if (!BuildBernsteinFacet(
          first_accepted, first_prepared, first_coefficients,
          duration, &first) ||
      !BuildBernsteinFacet(
          second_accepted, second_prepared, second_coefficients,
          duration, &second))
    return result;
  result.status = SubdivideSeparation(
      first, second, first_thickness, second_thickness,
      0, max_depth, max_work, &result.work, &result.deepest);
  return result;
}

RigidMemberSweepStatus BuildRigidMemberSweepBounds(
    Vec3 accepted_member, Vec3 prepared_member,
    const tl::fea::NodalRigidGroupSnapshot& accepted_group,
    const tl::fea::NodalRigidGroupSnapshot& prepared_group,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, SelfContactSweptParentBounds* output) noexcept {
  using R = RigidMemberSweepStatus;
  if (!output) return R::InvalidInput;
  const auto validation = ValidateRigidPolynomial(
      accepted_member, prepared_member, accepted_group, prepared_group,
      trajectory, duration);
  if (validation != R::Ok) return validation;

  const auto omega = prepared_group.state.omega;

  Interval arm[3], w[3];
  for (unsigned component = 0; component < 3; ++component) {
    if (!Difference(
            Component(accepted_member, component),
            Component(accepted_group.state.center, component),
            arm + component))
      return R::NonfiniteResult;
    const double value = Component(omega, component);
    w[component] = {value, value};
  }
  Interval first_cross[3], second_cross[3];
  if (!Cross(w, arm, first_cross) ||
      !Cross(w, first_cross, second_cross))
    return R::NonfiniteResult;

  double duration_squared = duration * duration;
  if (!std::isfinite(duration_squared))
    return R::NonfiniteResult;
  if (duration_squared != 0)
    duration_squared = Up(duration_squared);
  else
    duration_squared = std::numeric_limits<double>::denorm_min();

  SelfContactSweptParentBounds next;
  for (unsigned component = 0; component < 3; ++component) {
    const double curvature = std::max(
        std::fabs(second_cross[component].lower),
        std::fabs(second_cross[component].upper));
    double deviation = 0;
    if (curvature != 0) {
      deviation = duration_squared * curvature;
      if (!std::isfinite(deviation))
        return R::NonfiniteResult;
      deviation = Up(deviation);
      deviation = Up(deviation * 0.125);
      if (!std::isfinite(deviation))
        return R::NonfiniteResult;
    }
    const double first = Component(accepted_member, component);
    const double second = Component(prepared_member, component);
    const double lower = std::min(first, second) - deviation;
    const double upper = std::max(first, second) + deviation;
    if (!std::isfinite(lower) || !std::isfinite(upper))
      return R::NonfiniteResult;
    SetComponent(&next.lower, component, Down(lower));
    SetComponent(&next.upper, component, Up(upper));
  }
  if (!IsFinite(next.lower) || !IsFinite(next.upper))
    return R::NonfiniteResult;

  // Proof.  The authenticated owner certificate defines, for u in [0,1],
  //
  // x(u) = (1-u)x0 + u*x1
  //        - .5*u*(1-u)*h^2*q,
  // q = w x (w x (x0-c0)).
  //
  // This is the center/relative form stated on NodalPreparedView after its
  // linear center terms cancel.  It contains x0 and x1 exactly, is nonlinear
  // whenever the owner's constant prepared spin has nonzero centripetal
  // curvature, and reduces to the explicit ordinary rigid-member recurrence
  // in exact arithmetic.  Since 0 <= u(1-u) <= 1/4, each coordinate differs
  // from its endpoint chord by at most h^2*|q_i|/8.  The interval cross
  // products enclose exact-real q for the binary64 inputs: one nextafter
  // surrounds every rounded product, sum and difference.  Directed h^2,
  // deviation and final-face rounding therefore make next an enclosure, never
  // a crossing certificate.  Overflow, nonfinite input and rotations outside
  // the owner's admitted minor-arc domain fail closed before publication.
  *output = next;
  return R::Ok;
}

}  // namespace tlfea::contact::self_contact_transaction
