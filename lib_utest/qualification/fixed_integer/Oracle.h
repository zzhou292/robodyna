// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <gtest/gtest.h>
#include <vector>
namespace fixed_integer_test {
using Big = boost::multiprecision::cpp_int;
Integer Input(Big value);
Big Value(const Integer& value);
std::vector<Case> Corpus();
void CheckOracle(const Case&, const Result&);
void Same(const Result&, const Result&);
}  // namespace fixed_integer_test
