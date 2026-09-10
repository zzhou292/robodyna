#include "NodalWallContact.h"
#include "Q4ParametricContact.h"
#include <algorithm>
#include <new>

namespace tlfea::contact {
namespace {
bool Less(const NodalWallParentWeight& a,const NodalWallParentWeight& b) {
  if (a.parent_element_id!=b.parent_element_id) return a.parent_element_id<b.parent_element_id;
  if (a.parent_face_id!=b.parent_face_id) return a.parent_face_id<b.parent_face_id;
  return a.feature_id<b.feature_id;
}
bool Accumulate(Q4CertifiedIntegral& sum,Q4CertifiedIntegral value) {
  Q4IntegralInterval truth;
  return q4_bounds::Add({sum.lower,sum.upper},{value.lower,value.upper},&truth) &&
      q4_bounds::Certify(sum.value+value.value,truth,&sum);
}
} // namespace

NodalWallReport NodalWallWeights::Initialize(std::uint32_t global_nodes,
    const NodalWallParentInput* input,std::uint32_t count) {
  return Initialize(global_nodes,input,count,{MaxNodalWallParents,MaxNodalWallNodes,4*1024*1024});
}
NodalWallReport NodalWallWeights::Initialize(std::uint32_t global_nodes,
    const NodalWallParentInput* input,std::uint32_t count,const NodalWallWeightLimits& limits) {
  using Code=NodalWallStatus;
  if (!global_nodes || !input || !count) return {};
  if (!limits.max_nodes || !limits.max_parents || !limits.max_owned_bytes ||
      limits.max_nodes>MaxNodalWallWeightNodes || limits.max_parents>MaxNodalWallWeightParents)
    return {};
  if (global_nodes>limits.max_nodes || count>limits.max_parents)
    return nodal_wall_detail::Report(Code::Capacity);
  // Hard count bounds above make every product/sum below representable. Check
  // the full owned payload before allocation or reading any borrowed parent.
  const auto bytes=sizeof(*this)+decltype(parents_)::ExtraBytes(count)+decltype(nodes_)::ExtraBytes(global_nodes);
  if (bytes>limits.max_owned_bytes) return nodal_wall_detail::Report(Code::Capacity);
  try {
  NodalWallWeights next; next.global_node_count_=global_nodes; next.parent_count_=count;
  next.parents_.Resize(count); next.nodes_.Resize(global_nodes);
  for (unsigned p=0;p<count;++p) {
    auto failure=nodal_wall_detail::Report(Code::InvalidReference); failure.parent=p;
    const auto& in=input[p]; auto& out=next.parents_[p];
    if ((in.q4==nullptr)==(in.t3==nullptr)) return failure;
    if (in.q4) {
      if (!in.q4->prepared() || in.q4->node_count()!=global_nodes || in.q4_parent>=in.q4->parent_count()) return failure;
      const auto& reference=in.q4->parent(in.q4_parent);
      if (!reference.intrinsic.prepared()) return failure;
      const auto& parent=reference.intrinsic.parent();
      out.area=reference.area; out.arity=4; out.family=NodalWallParentFamily::Q4CenterArea;
      out.parent_element_id=parent.parent_element_id; out.parent_face_id=parent.parent_face_id; out.feature_id=parent.feature_id;
      for (unsigned n=0;n<4;++n) out.nodes[n]=parent.nodes[n];
    } else {
      if (in.q4_parent!=0 || !in.t3->prepared()) return failure;
      const auto& parent=in.t3->parent();
      if (!q4_bounds::Certify(.5*in.t3->density().value,in.t3->area_enclosure(),&out.area)) return failure;
      out.arity=3; out.family=NodalWallParentFamily::T3Native;
      out.parent_element_id=parent.parent_element_id; out.parent_face_id=parent.parent_face_id; out.feature_id=parent.feature_id;
      for (unsigned n=0;n<3;++n) out.nodes[n]=parent.nodes[n];
    }
    if (!nodal_wall_detail::Certificate(out.area,true)) return failure;
    for (unsigned n=0;n<out.arity;++n) {
      if (out.nodes[n]>=global_nodes) return failure;
      for (unsigned j=0;j<n;++j) if (out.nodes[j]==out.nodes[n]) return failure;
    }
    Q4IntegralInterval share;
    if (!q4_bounds::DividePositive({out.area.lower,out.area.upper},out.arity,&share) ||
        !q4_bounds::Certify(out.area.value/out.arity,share,&out.share) ||
        !nodal_wall_detail::Certificate(out.share,true)) return failure;
  }
  std::sort(next.parents_.data(),next.parents_.data()+count,Less);
  for (unsigned p=0;p<count;++p) {
    const auto& parent=next.parents_[p];
    for (unsigned j=0;j<p;++j) {
      const auto& other=next.parents_[j];
      if ((parent.parent_element_id==other.parent_element_id && parent.parent_face_id==other.parent_face_id) ||
          parent.feature_id==other.feature_id) {
        auto failure=nodal_wall_detail::Report(Code::DuplicateParent); failure.parent=p; return failure;
      }
    }
    if (!Accumulate(next.total_area_,parent.area)) return nodal_wall_detail::Report(Code::NonFiniteArithmetic);
  }
  for (unsigned node=0;node<global_nodes;++node) {
    NodalWallNodeWeight weight; weight.node=node; bool present=false;
    for (unsigned p=0;p<count;++p) for (unsigned n=0;n<next.parents_[p].arity;++n)
      if (next.parents_[p].nodes[n]==node) {
        present=true;
        if (!Accumulate(weight.area,next.parents_[p].share))
          return nodal_wall_detail::Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node);
      }
    if (present) {
      if (!nodal_wall_detail::Certificate(weight.area,true)) return nodal_wall_detail::Report(Code::NonFiniteArithmetic);
      next.nodes_[next.node_count_++]=weight;
    }
  }
  next.prepared_=true; *this=next;
  return nodal_wall_detail::Report(Code::Ok,Status::kOk);
  } catch (const std::bad_alloc&) {
    return nodal_wall_detail::Report(Code::Capacity);
  }
}
} // namespace tlfea::contact
