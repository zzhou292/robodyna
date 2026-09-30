// Synthetic native geometry at vehicle counts, not an original Yaris model.
#pragma once
#include "../t3/mixed_binding/ShellBatchBindingFixture.h"
#include <vector>

namespace vehicle_shell_test {
namespace fe=tl::fea;
using Geometry=fe::ShellBatchBinding;
using shell_binding_test::Bytes;
using Status=fe::ShellBindingStatus;
struct Fixture {
  std::vector<fe::ShellQephBindingInput> q;
  std::vector<fe::ShellT3BindingInput> t;
  std::size_t count;
  static constexpr std::size_t SourceQ=328344,SourceT=21301,SourceNodes=359785;
  explicit Fixture(std::size_t qc=2049,std::size_t tc=73,std::size_t nc=4097)
      :q(qc),t(tc),count(nc) {
    const auto squares=(count-1)/4;
    for(std::size_t i=0;i<q.size();++i) {
      const auto n=4*(i%squares); auto& p=q[i];
      p.nodes={n,n+1,n+2,n+3};p.source_parent_id=(std::uint64_t{1}<<58)+q.size()-i;
      Set(p.reference,p.nodes,1024.,1./32);
    }
    for(std::size_t i=0;i<t.size();++i) {
      const auto n=4*(i%squares);auto& p=t[i];
      p.nodes=i+1==t.size()?std::array<std::size_t,3>{count-4,count-3,count-1}:
                                    std::array<std::size_t,3>{n,n+1,n+2};
      p.source_parent_id=(std::uint64_t{1}<<60)+t.size()-i;
      Set(p.reference,p.nodes,7890.,1./128);
    }
  }
  std::uint64_t NodeId(std::size_t n) const {
    return n+1==count?(std::uint64_t{1}<<60)+17:(std::uint64_t{1}<<30)-n;
  }
  tl::math::Vec3 Position(std::size_t n) const {
    if(n+1==count) return {2.*double((count-1)/4),0.,-0.};
    const auto corner=n%4;
    return {2.*double(n/4)+double(corner==1||corner==2),double(corner>=2),n%2?0.:-0.};
  }
  template<class Input,std::size_t N> void Set(Input& input,
      const std::array<std::size_t,N>& nodes,double density,double thickness) {
    input.density=density;input.thickness=thickness;input.young_modulus=2e6;input.poisson_ratio=.3;
    for(std::size_t i=0;i<N;++i) {
      input.node_ids[i]=static_cast<decltype(input.node_ids[i]+0)>(NodeId(nodes[i]));
      input.position[i]=Position(nodes[i]);
    }
  }
  fe::ShellBatchCollectionInput input() const {return {q.data(),t.data(),q.size(),t.size(),count};}
};

template<class Reference,class Parent> void AddExpected(const Reference& ref,const Parent& parent,
    std::vector<fe::ShellBindingMass>& nodes,fe::ShellBindingMass& total) {
  for(std::size_t n=0;n<parent.nodes.size();++n) {
    const fe::ShellBindingMass term{ref.nodal_mass[n],ref.isotropic_inertia[n],
                                   ref.physical_inertia[n],ref.added_inertia[n]};
    shell_binding_test::AddExact(nodes[parent.nodes[n]],term);shell_binding_test::AddExact(total,term);
  }
}
} // namespace vehicle_shell_test
