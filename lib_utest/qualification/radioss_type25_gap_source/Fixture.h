// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>
namespace gap_source_test {
template<class T> const T* Data(const std::vector<T>& value) { return value.empty()?nullptr:value.data(); }
template<class T> T* Data(std::vector<T>& value) { return value.empty()?nullptr:value.data(); }
struct Fixture {
  std::size_t nodes=12;
  g::Profile profile{1,0,1,1,0,0,1.,1.e30,1.e30};
  std::vector<g::PhysicalShell> shells;
  std::vector<g::Line> trusses,beams;
  std::vector<g::Spring> springs;
  std::vector<n::startup::Main> mains;
  std::vector<std::uint32_t> secondary,main_nodes;
  std::size_t primary=1;
  Fixture() {
    g::PhysicalShell q;q.source_element_id=1;q.layout=n::ShellLayout::Quad4;
    q.young=100;q.structural_thickness=2;q.property_thickness=2;
    for(unsigned k=0;k<4;++k)q.nodes[k]=k;
    shells.push_back(q);
    q.source_element_id=2;q.property_thickness=8;
    for(unsigned k=0;k<4;++k)q.nodes[k]=k+3;
    shells.push_back(q);
    q.source_element_id=3;q.layout=n::ShellLayout::Triangle3;q.property_thickness=4;
    q.nodes[0]=6;q.nodes[1]=7;q.nodes[2]=q.nodes[3]=8;shells.push_back(q);
    g::Line beam;beam.source_element_id=9;beam.nodes[0]=4;beam.nodes[1]=9;beam.native_area=36;beams.push_back(beam);
    g::Spring spring;spring.native_element_id=100;spring.property_type=13;spring.nodes[0]=5;spring.nodes[1]=10;
    springs.push_back(spring);
    mains.resize(2);mains[0].segment_type=2;mains[1].segment_type=-1;
    const unsigned reverse[]{1,0,3,2};
    for(unsigned k=0;k<4;++k){mains[0].nodes[k]=k;mains[1].nodes[k]=reverse[k];}
    for(unsigned k=0;k<nodes;++k)secondary.push_back(k);
    main_nodes={0,1,2,3};
  }
  g::Input Input() const {
    g::Input in;in.profile=profile;in.node_count=nodes;in.shells=Data(shells);in.shell_count=shells.size();
    in.trusses=Data(trusses);in.truss_count=trusses.size();in.beams=Data(beams);in.beam_count=beams.size();
    in.springs=Data(springs);in.spring_count=springs.size();in.mains=Data(mains);in.main_count=mains.size();in.primary_count=primary;
    in.secondary_nodes=Data(secondary);in.secondary_count=secondary.size();in.main_nodes=Data(main_nodes);in.main_node_count=main_nodes.size();
    return in;
  }
};
inline std::uint64_t Bits(double x) { std::uint64_t b;std::memcpy(&b,&x,sizeof(b));return b; }
inline void Exact(double a,double b) { EXPECT_EQ(Bits(a),Bits(b)); }
inline void Same(const Result& a,const Result& b) {
  ASSERT_EQ(a.secondary.size(),b.secondary.size());ASSERT_EQ(a.main_nodes.size(),b.main_nodes.size());ASSERT_EQ(a.mains.size(),b.mains.size());
  for(unsigned i=0;i<a.secondary.size();++i)Exact(a.secondary[i],b.secondary[i]);
  for(unsigned i=0;i<a.main_nodes.size();++i)Exact(a.main_nodes[i],b.main_nodes[i]);
  for(unsigned i=0;i<a.mains.size();++i) {
    for(unsigned k=0;k<4;++k)Exact(a.mains[i].corner[k],b.mains[i].corner[k]);
    Exact(a.mains[i].maximum,b.mains[i].maximum);
  }
  Exact(a.minimum_secondary,b.minimum_secondary);Exact(a.maximum_secondary,b.maximum_secondary);
}
struct Attempt {
  g::Forecast forecast;
  tl::util::HostArena scratch;
  Result result;
  explicit Attempt(const g::Input& in) {
    const auto status=g::Preflight(in,{},forecast);
    if(status.status!=g::Status::Ok || !scratch.Initialize(forecast.scratch_bytes))
      throw std::runtime_error("Gap fixture forecast/allocation");
    result.secondary.assign(in.secondary_count,71.);result.main_nodes.assign(in.main_node_count,-31.);
    result.mains.resize(in.main_count);
    for(auto& m:result.mains) {for(auto& x:m.corner)x=91.;m.maximum=-17.;}
  }
  g::Output Output() {
    return {Data(result.secondary),result.secondary.size(),Data(result.main_nodes),result.main_nodes.size(),Data(result.mains),result.mains.size()};
  }
  g::Report Run(const g::Input& in,g::Limits limits={}) {
    const auto report=g::Build(in,limits,scratch.data(),scratch.bytes(),Output());
    if(report.status==g::Status::Ok) {
      result.minimum_secondary=report.minimum_secondary;result.maximum_secondary=report.maximum_secondary;
    }
    return report;
  }
};
inline void Compare(const Fixture& f) {
  const auto in=f.Input();Attempt actual(in);const auto report=actual.Run(in);
  ASSERT_EQ(report.status,g::Status::Ok);ASSERT_TRUE(report.completed);
  Same(actual.result,Oracle(in));
}
} // namespace gap_source_test
