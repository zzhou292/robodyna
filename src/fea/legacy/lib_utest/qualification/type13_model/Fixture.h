// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../type13/Fixture.h"
#include "lib_src/elements/type13/Type13Model.h"
#include <gtest/gtest.h>
#include <vector>

namespace type13_model_test {
namespace t=tl::fea::type13;
struct Fixture {
  type13_test::Fixture material;
  std::vector<t::ModelNode> nodes{{10,0,{1200,-400,700}},
      {20,1,{1201.25,-398,702.5}},{30,2,{1203,-396,704}},{99,SIZE_MAX,{0,0,0}}};
  std::vector<t::ModelConnection> connections{{100,0,{0,1,3}},{200,0,{1,2,3}}};
  t::ModelPropertyInput declaration{50,material.Input()};
  t::ModelInput Input() const { return {1,{1000,.001,1},nodes.data(),&declaration,
      connections.data(),nodes.size(),1,connections.size(),3}; }
};
} // namespace type13_model_test
