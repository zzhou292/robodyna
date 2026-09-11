// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/constraints/tied_shell/TiedClassification.h"
#include <gtest/gtest.h>
#include <vector>

namespace classification_test {
namespace tied=tl::constraints::tied_shell;
using namespace tied;
template<class T> ClassificationView<T> View(const std::vector<T>& v) { return {v.data(),v.size()}; }
struct Role {
  std::uint32_t id=100;
  int type=2,level=28;
  std::vector<std::uint32_t> slaves,masters;
};
struct Section { int type=100; std::vector<std::uint32_t> nodes; };
struct Fixture {
  std::vector<ClassificationNode> nodes;
  std::vector<Role> roles;
  std::vector<Section> sections;
  std::vector<std::int32_t> cyclic,tags;
  std::vector<ClassificationTetraEdge> tetra;
  std::vector<std::uint32_t> rbe2;
  std::vector<std::vector<std::uint32_t>> rbe3,groups;
  std::vector<ClassificationInterface> interface_views;
  std::vector<ClassificationSection> section_views;
  std::vector<ClassificationNodes> rbe3_views;
  std::vector<ClassificationRigidGroup> group_views;
  explicit Fixture(std::size_t n=12):nodes(n) {
    for(std::size_t i=0;i<n;++i) nodes[i].source_id=static_cast<std::uint32_t>(1000+i);
    roles.push_back({100,2,28,{0},{static_cast<std::uint32_t>(n-1)}});
  }
  ClassificationContext Context() const { return {77,View(nodes)}; }
  ClassificationInput Input() {
    interface_views.clear();
    for(const auto& r:roles) interface_views.push_back({r.id,r.type,r.level,View(r.slaves),View(r.masters)});
    section_views.clear();
    for(const auto& s:sections) section_views.push_back({s.type,View(s.nodes)});
    rbe3_views.clear();
    for(const auto& r:rbe3) rbe3_views.push_back(View(r));
    return {Context(),View(interface_views),View(section_views),View(cyclic),View(tetra),View(tags),View(rbe2),View(rbe3_views)};
  }
  RigidRegistrationInput Rigid(int ikrem=0,int iddlevel=0) {
    group_views.clear();
    for(std::size_t i=0;i<groups.size();++i)
      group_views.push_back({static_cast<std::uint32_t>(200+i),View(groups[i])});
    return {Context(),View(group_views),iddlevel,ikrem};
  }
};
inline void SameKinematics(const NativeKinematics& a,const NativeKinematics& b) {
  EXPECT_EQ(a.conditions,b.conditions);
  EXPECT_EQ(a.translation,b.translation);
  EXPECT_EQ(a.rotation,b.rotation);
  EXPECT_EQ(a.duplicate_conditions,b.duplicate_conditions);
  EXPECT_EQ(a.incompatible_conditions,b.incompatible_conditions);
}
inline void Same(const ClassificationResult& a,const ClassificationResult& b) {
  ASSERT_EQ(a.phase(),b.phase());
  ASSERT_EQ(a.source_instance_id(),b.source_instance_id());
  ASSERT_EQ(a.nodes().count,b.nodes().count);
  for(std::size_t i=0;i<a.nodes().count;++i) {
    EXPECT_EQ(a.nodes().data[i].source_id,b.nodes().data[i].source_id);
    SameKinematics(a.nodes().data[i].kinematics,b.nodes().data[i].kinematics);
  }
  ASSERT_EQ(a.interfaces().count,b.interfaces().count);
  for(std::size_t i=0;i<a.interfaces().count;++i) {
    const auto x=a.interfaces().data[i],y=b.interfaces().data[i];
    EXPECT_EQ(x.source_id,y.source_id); EXPECT_EQ(x.native_type,y.native_type);
    EXPECT_EQ(x.level,y.level); EXPECT_EQ(x.selected,y.selected);
    EXPECT_EQ(x.slave_offset,y.slave_offset); EXPECT_EQ(x.slave_count,y.slave_count);
  }
  ASSERT_EQ(a.irupt().count,b.irupt().count);
  ASSERT_EQ(a.slave_nodes().count,b.slave_nodes().count);
  for(std::size_t i=0;i<a.slave_nodes().count;++i) EXPECT_EQ(a.slave_nodes().data[i],b.slave_nodes().data[i]);
  for(std::size_t i=0;i<a.irupt().count;++i) EXPECT_EQ(a.irupt().data[i],b.irupt().data[i]);
  ASSERT_EQ(a.interface_decode().count,b.interface_decode().count);
  for(std::size_t i=0;i<a.interface_decode().count;++i)
    ASSERT_EQ(a.interface_decode().data[i],b.interface_decode().data[i])<<i;
  EXPECT_EQ(a.native_kinset_warnings(),b.native_kinset_warnings());
  EXPECT_EQ(a.native_penalty_warnings(),b.native_penalty_warnings());
}
} // namespace classification_test
