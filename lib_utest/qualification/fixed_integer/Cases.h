// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/FixedInteger.h"
#include <cstddef>
namespace fixed_integer_test {
using Arithmetic = tl::math::fixed_integer::Arithmetic<8>;
using Integer = Arithmetic::Integer;
using CheckedSign = tl::math::fixed_integer::CheckedSign;
inline constexpr std::size_t MaximumCases = 2048;
enum class Operation : unsigned { Add, Subtract, Multiply, Negate, Shift, Scalar, Compare, Sign };
struct Case {
  Operation operation{};
  Integer a, b;
  std::uint64_t scalar = 0;
  unsigned shift = 0;
};
struct Result { Integer value; CheckedSign sign; };
TL_MATH_HOST_DEVICE inline Result Evaluate(const Case& row) noexcept {
  Result result;
  switch (row.operation) {
    case Operation::Add: result.value=Arithmetic::Add(row.a,row.b); break;
    case Operation::Subtract: result.value=Arithmetic::Subtract(row.a,row.b); break;
    case Operation::Multiply: result.value=Arithmetic::Multiply(row.a,row.b); break;
    case Operation::Negate: result.value=Arithmetic::Negate(row.a); break;
    case Operation::Shift: result.value=Arithmetic::ShiftLeft(row.a,row.shift); break;
    case Operation::Scalar: result.value=Arithmetic::FromU64(row.scalar); break;
    case Operation::Compare: result.sign=Arithmetic::Compare(row.a,row.b); return result;
    case Operation::Sign: result.sign=Arithmetic::Result(row.a); return result;
  }
  result.sign=Arithmetic::Result(result.value);
  return result;
}
}  // namespace fixed_integer_test
