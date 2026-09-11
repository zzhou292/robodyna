// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25Model.h"
#include "Type25Identity.h"
#include "../../../lib_utils/SourceIdentityIndex.h"
#include <algorithm>

namespace tl::fea::type25::detail {
// Vehicle startup only. Sorted indexes retain first source occurrences;
// coefficient production still traverses the original connection/endpoint order.
class ModelIndex {
 public:
  using Index=tl::util::SourceIdentityIndex<1>;
  static constexpr std::size_t Bytes(std::size_t properties,std::size_t connections) noexcept {
    return Index::Bytes(properties)+Index::Bytes(connections)+2*Index::Bytes(2*connections);
  }
  void Prepare(const ModelInput& in) {
    properties_.Prepare(in.property_count,[&](auto p){return in.properties[p].source_property_id;});
    elements_.Prepare(in.connection_count,[&](auto e){return in.connections[e].source_element_id;});
    nodes_.Prepare(2*in.connection_count,[&](auto e){return in.connections[e/2].source_node_id[e%2];});
    globals_.Prepare(2*in.connection_count,[&](auto e){return in.connections[e/2].global_node[e%2];});
  }
  bool FirstProperty(const ModelInput& in,std::size_t p) const noexcept {
    return properties_.First(in.properties[p].source_property_id)==p;
  }
  // Input identity checks are performed by the existing owning caller first.
  ModelReport CheckPriorConnections(const ModelInput& in,std::size_t i) const noexcept {
    const auto& c=in.connections[i];
    const auto element=elements_.First(c.source_element_id);
    std::size_t conflict=SIZE_MAX;
    for(unsigned a=0;a<2;++a) {
      const auto by_node=nodes_.First(c.source_node_id[a]);
      const auto by_global=globals_.First(c.global_node[a]);
      for(auto occurrence:{by_node,by_global}) {
        if(occurrence==SIZE_MAX||occurrence/2>=i)continue;
        const auto& p=in.connections[occurrence/2];const auto b=occurrence%2;
        const bool same_index=c.global_node[a]==p.global_node[b],same_id=c.source_node_id[a]==p.source_node_id[b];
        if(same_index!=same_id||(same_index&&!Same(c.position[a],p.position[b])))
          conflict=std::min(conflict,occurrence/2);
      }
    }
    // Original nested loop checks element identity before endpoint consistency
    // within each preceding connection. Retain that exact error precedence.
    if(element<i&&element<=conflict)return {Status::DuplicateIdentity,"Duplicate source connection ID",i};
    if(conflict!=SIZE_MAX)return {Status::DuplicateIdentity,"Shared endpoint source identity or exact coordinate mismatch",i};
    return {Status::Success,"",i};
  }
 private:
  Index properties_,elements_,nodes_,globals_;
};
} // namespace tl::fea::type25::detail
