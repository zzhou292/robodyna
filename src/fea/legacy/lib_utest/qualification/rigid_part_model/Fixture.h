// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/constraints/NodalRigidPartAssemblyModel.h"
#include "../qbat_binding/Fixture.h"
#include <numeric>

namespace rigid_part_model_test {
namespace fe=tl::fea;
namespace r=fe::rigid;
using Model=r::NodalRigidPartAssemblyModel;
using qbat_binding_test::Bits;
using qbat_binding_test::Bytes;
struct Fixture {
  qbat_binding_test::Fixture shell_input;
  fe::ShellBatchBinding shells;
  std::vector<fe::NodalDomainNode> nodes;
  std::vector<fe::ElementMassSource> point_rows;
  std::vector<std::vector<std::uint64_t>> part,extra;
  std::vector<r::PartTopologyMerge> merges;
  fe::NodalRigidSourceUnits units{1000,.001};
  explicit Fixture(std::vector<std::size_t> counts={4,4,4,4}) {
    EXPECT_EQ(shells.InitializeFormulations(shell_input.Input()).status,fe::ShellBindingStatus::Success);
    for(const auto& n:shells.active_nodes()) nodes.push_back({n.source_id,n.position});
    part.resize(counts.size());extra.resize(counts.size());
    std::size_t added=0;
    auto append=[&](std::vector<std::uint64_t>& list) {
      const auto i=added++;
      const auto node=nodes.size();
      const auto id=10000+i;
      nodes.push_back({id,{.003*double(i%7)+.1,.002*double((i/7)%7)-.2,.005*double(i/49)+.3}});
      point_rows.push_back({90000+i,id,node,.0001+double(i%11)*.00001});
      list.push_back(id);
    };
    for(std::size_t p=0;p<counts.size();++p) {
      for(std::size_t n=0;n<counts[p];++n) {
        if(p==0&&n<4) part[p].push_back(nodes[n].source_id);
        else append(part[p]);
      }
      if(p%2==0) append(extra[p]);
    }
    for(std::size_t p=0;p+1<counts.size()&&merges.size()<2;p+=2)
      merges.push_back({200+p,201+p});
  }
  fe::NodalCoefficientLedger Ledger(bool include_points=true) const {
    fe::NodalNodeDomain domain;
    EXPECT_TRUE(domain.Initialize({1,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
    fe::ShellNodeMap map;
    EXPECT_TRUE(map.Initialize(shells,domain));
    fe::ElementMassContributions points;
    EXPECT_TRUE(points.Initialize(domain,{1,1000,point_rows.data(),point_rows.size()}));
    fe::NodalCoefficientLedger result;
    EXPECT_TRUE(result.InitializeWithElementMass({{&map,nullptr,nullptr},include_points?&points:nullptr},
        fe::CoefficientLimits::Vehicle()));
    return result;
  }
  std::unique_ptr<r::NodalRigidPartTopology> Topology(std::uint64_t instance=1) const {
    std::vector<r::PartTopologyPartInput> parts;
    std::vector<r::PartTopologyExtraInput> extras;
    std::vector<std::uint64_t> expected;
    for(std::size_t p=0;p<part.size();++p) {
      parts.push_back({200+p,part[p].data(),part[p].size()});
      expected.insert(expected.end(),part[p].begin(),part[p].end());
      if(!extra[p].empty()) {
        extras.push_back({200+p,400+p,extra[p].data(),extra[p].size()});
        expected.insert(expected.end(),extra[p].begin(),extra[p].end());
      }
    }
    r::PartTopologyInput input;
    input.source_instance_id=instance;
    input.parts=parts.data();input.part_count=parts.size();
    input.extras=extras.empty()?nullptr:extras.data();input.extra_count=extras.size();
    input.merges=merges.empty()?nullptr:merges.data();input.merge_count=merges.size();
    input.expected_members=expected.data();input.expected_member_count=expected.size();
    auto result=std::make_unique<r::NodalRigidPartTopology>();
    EXPECT_TRUE(result->Initialize(input));
    return result;
  }
};
inline std::array<double,13> RawValues(const r::AssemblyRawBody& b) {
  return {b.mass,b.center.x,b.center.y,b.center.z,b.tensor.v[0],b.tensor.v[1],b.tensor.v[2],
    b.tensor.v[3],b.tensor.v[4],b.tensor.v[5],b.tensor.v[6],b.tensor.v[7],b.tensor.v[8]};
}
// Independent supplied packet assembled from source sets and ledger, never
// from the tested model's mapped members or generated primary/results.
struct Packet {
  std::vector<r::AssemblyMassPoint> part,extra;
  r::AssemblyPrimary primary{};
  fe::NodalRigidSourceUnits units;
  Packet(const Fixture& f,const fe::NodalCoefficientLedger& c,std::size_t p):units(f.units) {
    auto copy=[&](auto ids,auto& out) {
      std::sort(ids.begin(),ids.end());
      for(const auto id:ids) {
        const auto n=c.domain()->Find(id);
        const auto& row=c.nodes()[n].coefficients;
        out.push_back({c.domain()->nodes()[n].position,row.mass,row.isotropic_inertia});
      }
    };
    copy(f.part[p],part);copy(f.extra[p],extra);
    for(const auto& v:part) {
      primary.position.x+=v.position.x;primary.position.y+=v.position.y;primary.position.z+=v.position.z;
    }
    primary.position.x/=part.size();primary.position.y/=part.size();primary.position.z/=part.size();
    primary.mass=1e-20*units.mass_to_kg;
    primary.inertia=primary.mass*units.length_to_m*units.length_to_m;
  }
  r::AssemblyBodyInput Input() const {
    return {primary,part.data(),extra.empty()?nullptr:extra.data(),part.size(),extra.size(),units};
  }
};
} // namespace rigid_part_model_test
