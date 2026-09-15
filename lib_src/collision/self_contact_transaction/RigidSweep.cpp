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

RigidMemberSweepStatus CertifyRigidPointAffineMotion(
    const WeightedSurfacePoint& point,
    VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, bool* affine) noexcept {
  using R = RigidMemberSweepStatus;
  if (!affine || !node_rigid_groups ||
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
  return R::Ok;
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
  using R = RigidMemberSweepStatus;
  if (!affine) return R::InvalidInput;
  *affine = false;
  bool all_affine = true;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    bool point_affine = false;
    const auto status = CertifyRigidPointAffineMotion(
        facet.vertices[vertex], accepted, prepared,
        node_rigid_groups, accepted_groups, prepared_groups,
        group_count, trajectory, duration, &point_affine);
    if (status != R::Ok) return status;
    all_affine = all_affine && point_affine;
  }
  *affine = all_affine;
  return R::Ok;
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
