#pragma once
#include "GroupTestSupport.h"
#include <vector>
namespace vehicle_rigid_test {
namespace fe=tl::fea;namespace rigid=fe::rigid;
using Vec3=tl::math::Vec3;
struct Declaration {std::uint64_t id;std::size_t count;bool internal;};
inline constexpr Declaration Declarations[]{
#include "YarisRigidGroupCountRows.inc"
};
inline constexpr std::size_t VehicleNodes=359785;
inline constexpr std::uint64_t Source=0x59524947434150;
// Original source group-count/ID shape only. All node IDs, geometry and native
// coefficients here are synthetic. No crossing source group is being truncated
// or mechanically admitted by this capacity fixture.
struct SourceFixture {
  std::vector<fe::NodalRigidGroupMember> members;
  std::vector<fe::NodalRigidGroupInput> groups;
  fe::NodalRigidGroupModel model;
  explicit SourceFixture(bool internal_only=false,std::size_t maximum_groups=0,std::size_t nodes=VehicleNodes) {
    std::vector<Declaration> selected;
    if(maximum_groups)for(std::size_t g=0;g<maximum_groups;++g)selected.push_back({8000000+g,8,true});
    else for(const auto d:Declarations)if(!internal_only||d.internal)selected.push_back(d);
    std::size_t count=0;for(const auto d:selected)count+=d.count;
    members.reserve(count);groups.reserve(selected.size());
    for(std::size_t g=0;g<selected.size();++g) {
      const auto d=selected[g];const auto offset=members.size();
      for(std::size_t m=0;m<d.count;++m) {
        const auto index=members.size(),node=index+1==count?nodes-1:index;
        members.push_back({10000000000ULL+index,node,{.125*g+.001*m,.002*(m%3),.003*(m%5)},2,.001,.0004,.0006});
      }
      groups.push_back({d.id,d.id,members.data()+offset,d.count});
    }
  }
  fe::NodalRigidGroupModelInput Input(std::size_t nodes=VehicleNodes) const {
    return {Source,nodes,groups.data(),groups.size(),{1000,.001},fe::NodalRigidGroupLimits::Vehicle()};
  }
};
} // namespace vehicle_rigid_test
