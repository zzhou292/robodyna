// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/math/ScalarBits.h"
#include <gtest/gtest.h>
namespace type25_seed_test {
inline void SameSeed(const n::NativeNodalSeed& a, const n::NativeNodalSeed& b) {
  EXPECT_TRUE(tl::math::SameScalarBits(a.volume,b.volume));
  EXPECT_TRUE(tl::math::SameScalarBits(a.bulk_volume,b.bulk_volume));
  EXPECT_TRUE(tl::math::SameScalarBits(a.existing_stiffness,b.existing_stiffness));
}
struct SeedFixture {
  tl::util::HostArena scratch;
  std::vector<n::NativeNodalSeed> nodes;
  seed::Forecast forecast;
  seed::Report Build(const seed::Input& input) {
    auto report = seed::Preflight(input,{},forecast);
    if (report.status != seed::Status::Ok) return report;
    if ((!scratch.data() && !scratch.Initialize(forecast.scratch_bytes)) || scratch.bytes()<forecast.scratch_bytes) throw std::runtime_error("Seed fixture arena");
    nodes.assign(input.node_count,{});
    return seed::Accumulate(input,{},scratch.data(),forecast.scratch_bytes,{nodes.data(),nodes.size()});
  }
};
struct ShellFixture {
  tl::util::HostArena scratch;
  std::vector<shell::NodeFields> nodes;
  std::vector<std::uint32_t> mains;
  std::vector<double> main_values;
  std::vector<shell::Secondary> secondary;
  std::vector<shell::SecondaryFields> values;
  shell::Input input;
  shell::Forecast forecast;
  ShellFixture(const Prepared& rows,std::size_t count,shell::Population population) {
    input.profile = ShellProfile(population);
    input.node_count = count;
    input.shells = rows.shells.data(); input.shell_count = rows.shells.size();
    for (std::size_t i=0;i<count;++i) secondary.push_back({std::uint32_t(i),1.});
    nodes.resize(count); values.resize(count);
    Refresh();
  }
  void SelectMain(std::uint32_t row) {mains={row};main_values.resize(1);Refresh();}
  void Refresh() {
    input.primary_shells=mains.empty()?nullptr:mains.data();input.primary_count=mains.size();
    input.secondary=secondary.data();input.secondary_count=secondary.size();
  }
  shell::Output Output() {return {nodes.data(),nodes.size(),main_values.empty()?nullptr:main_values.data(),
      main_values.size(),values.data(),values.size()};}
  shell::Report Build(n::NativeNodalSeedView seed_view) {
    Refresh();
    const auto report=shell::Preflight(input,seed_view,{},forecast);
    if(report.status!=shell::Status::Ok)return report;
    if((!scratch.data() && !scratch.Initialize(forecast.scratch_bytes)) || scratch.bytes()<forecast.scratch_bytes)throw std::runtime_error("Shell fixture arena");
    return shell::Build(input,seed_view,{},scratch.data(),forecast.scratch_bytes,Output());
  }
};
}
