// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <array>
#include <climits>
#include <stdexcept>

extern "C" void native_classify(int,int,const int*,const int*,const int*,
    int,int,const int*,const int*,int,const int*,int,const int*,int,const int*,
    int,const int*,const int*,int,const int*,int,const int*,int,const int*,
    int*,int*,int*,int*,int*,int*);
extern "C" void native_rigid(int,int,const int*,const int*,const int*,int,
    const int*,int,int,int*,int*,int*,int*);

namespace classification_test {
namespace {
void Require(bool value) {
  if(!value) throw std::invalid_argument("Native classification packet outside fixture bounds");
}
int Size(std::size_t n) {
  Require(n<=4u*1024*1024);
  return static_cast<int>(n);
}
template<class T> void Range(ClassificationView<T> v) {
  Size(v.count);
  Require(!v.count || v.data);
}
void AppendNodes(std::vector<int>& out,ClassificationNodes nodes,std::size_t count) {
  Range(nodes);
  Require(nodes.count<=4u*1024*1024-out.size());
  for(std::size_t i=0;i<nodes.count;++i) {
    Require(nodes.data[i]<count);
    out.push_back(static_cast<int>(nodes.data[i]+1));
  }
}
struct Context {
  std::vector<int> ids,kin;
  explicit Context(ClassificationContext c) {
    Range(c.nodes);
    Require(c.nodes.count && c.nodes.count<=1024u*1024);
    ids.resize(c.nodes.count);
    kin.resize(5*c.nodes.count);
    for(std::size_t i=0;i<c.nodes.count;++i) {
      const auto& node=c.nodes.data[i];
      Require(node.source_id && node.source_id<=INT_MAX);
      ids[i]=static_cast<int>(node.source_id);
      const auto k=node.kinematics;
      const std::array<int,5> fields{{k.conditions,k.translation,k.rotation,
                                   k.duplicate_conditions,k.incompatible_conditions}};
      for(std::size_t field=0;field<5;++field) kin[field*c.nodes.count+i]=fields[field];
    }
  }
};
}
NativeResult NativeClassify(const ClassificationInput& in) {
  static_assert(sizeof(int)==4);
  Context context(in.context);
  const auto count=context.ids.size();
  Range(in.interfaces); Range(in.sections); Range(in.cyclic_tags);
  Range(in.tetra_edges); Range(in.tetra_tags); Range(in.rbe2_nodes); Range(in.rbe3_members);
  Require(in.interfaces.count && in.interfaces.count<=4096 && in.sections.count<=4096);
  Require(!in.cyclic_tags.count || in.cyclic_tags.count==count);
  Require(in.tetra_tags.count==(in.tetra_edges.count ? count : 0));
  std::vector<int> roles,slaves,masters,sections,section_nodes,cyclic(count),tetra,tags(count);
  std::vector<int> rbe2,rbe3_counts,rbe3_members;
  for(std::size_t i=0;i<in.interfaces.count;++i) {
    const auto& r=in.interfaces.data[i];
    Require(r.source_id && r.source_id<=INT_MAX);
    roles.insert(roles.end(),{static_cast<int>(r.source_id),r.native_type,r.level,Size(r.slaves.count),Size(r.masters.count)});
    AppendNodes(slaves,r.slaves,count); AppendNodes(masters,r.masters,count);
  }
  for(std::size_t i=0;i<in.sections.count;++i) {
    const auto& s=in.sections.data[i];
    sections.insert(sections.end(),{s.native_type,Size(s.nodes.count)});
    AppendNodes(section_nodes,s.nodes,count);
  }
  for(std::size_t i=0;i<in.cyclic_tags.count;++i) {
    Require(in.cyclic_tags.data[i]>=0 && in.cyclic_tags.data[i]<=INT_MAX-13);
    cyclic[i]=in.cyclic_tags.data[i];
  }
  for(std::size_t i=0;i<in.tetra_edges.count;++i) {
    const auto edge=in.tetra_edges.data[i];
    const std::vector<std::uint32_t> nodes{edge.midpoint,edge.first_corner,edge.second_corner};
    AppendNodes(tetra,View(nodes),count);
  }
  for(std::size_t i=0;i<in.tetra_tags.count;++i) tags[i]=in.tetra_tags.data[i];
  AppendNodes(rbe2,in.rbe2_nodes,count);
  for(std::size_t i=0;i<in.rbe3_members.count;++i) {
    rbe3_counts.push_back(Size(in.rbe3_members.data[i].count));
    AppendNodes(rbe3_members,in.rbe3_members.data[i],count);
  }
  NativeResult out;
  out.five_blocks.resize(context.kin.size()); out.irupt.resize(slaves.size()); out.itf.resize(8192);
  int status=-1;
  native_classify(Size(count),Size(in.interfaces.count),context.ids.data(),context.kin.data(),roles.data(),
      Size(slaves.size()),Size(masters.size()),slaves.data(),masters.data(),Size(in.sections.count),
      sections.data(),Size(section_nodes.size()),section_nodes.data(),in.cyclic_tags.count ? 1 : 0,cyclic.data(),
      Size(in.tetra_edges.count),tetra.data(),tags.data(),Size(rbe2.size()),rbe2.data(),
      Size(rbe3_counts.size()),rbe3_counts.data(),Size(rbe3_members.size()),rbe3_members.data(),
      out.five_blocks.data(),out.irupt.data(),out.itf.data(),&out.warnings,&out.penalties,&status);
  Require(status==0);
  return out;
}
NativeResult NativeRegister(const RigidRegistrationInput& in) {
  Context context(in.context);
  Range(in.groups);
  std::vector<int> counts,members;
  for(std::size_t i=0;i<in.groups.count;++i) {
    counts.push_back(Size(in.groups.data[i].members.count));
    AppendNodes(members,in.groups.data[i].members,context.ids.size());
  }
  NativeResult out;
  out.five_blocks.resize(context.kin.size()); out.itf.resize(8192);
  int status=-1;
  native_rigid(Size(context.ids.size()),Size(counts.size()),context.ids.data(),context.kin.data(),
      counts.data(),Size(members.size()),members.data(),in.native_iddlevel,in.native_ikrem,
      out.five_blocks.data(),out.itf.data(),&out.warnings,&status);
  Require(status==0);
  return out;
}
void Compare(const ClassificationResult& actual,const NativeResult& native) {
  const auto count=actual.nodes().count;
  ASSERT_EQ(native.five_blocks.size(),5*count);
  for(std::size_t i=0;i<count;++i) {
    SCOPED_TRACE(i);
    const auto k=actual.nodes().data[i].kinematics;
    EXPECT_EQ(k.conditions,native.five_blocks[i]);
    EXPECT_EQ(k.translation,native.five_blocks[count+i]);
    EXPECT_EQ(k.rotation,native.five_blocks[2*count+i]);
    EXPECT_EQ(k.duplicate_conditions,native.five_blocks[3*count+i]);
    EXPECT_EQ(k.incompatible_conditions,native.five_blocks[4*count+i]);
  }
  ASSERT_EQ(actual.irupt().count,native.irupt.size());
  for(std::size_t i=0;i<native.irupt.size();++i) EXPECT_EQ(actual.irupt().data[i],native.irupt[i]);
  ASSERT_EQ(actual.interface_decode().count,native.itf.size());
  for(std::size_t i=0;i<native.itf.size();++i) ASSERT_EQ(actual.interface_decode().data[i],native.itf[i])<<i;
  EXPECT_EQ(actual.native_kinset_warnings(),static_cast<std::uint64_t>(native.warnings));
  EXPECT_EQ(actual.native_penalty_warnings(),static_cast<std::uint64_t>(native.penalties));
}
} // namespace classification_test
