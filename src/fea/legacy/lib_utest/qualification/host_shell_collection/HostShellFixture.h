#pragma once
// Synthetic assembly-sized coverage, not authenticated Yaris mechanics.
#include "../plasticity_binding/PlasticityBindingFixture.h"
#include "../t3/mixed_binding/ShellBatchBindingFixture.h"
#include <vector>

namespace host_shell_test {
namespace fe=tl::fea;
using Geometry=fe::ShellBatchBinding;
using Catalog=fe::ShellBatchPlasticityBinding;
using shell_binding_test::Bytes;
constexpr std::size_t QCount=804,TCount=111,NodeCount=1030;
struct Fixture {
  std::vector<fe::ShellQephBindingInput> q{QCount};
  std::vector<fe::ShellT3BindingInput> t{TCount};
  std::array<double,46> x0{},y0{};
  std::array<double,17> x1{},y1{};
  std::array<fe::ShellPlasticityCurveInput,2> curves;
  std::array<fe::ShellPlasticityMaterialInput,6> materials;
  std::array<fe::ShellPlasticitySectionInput,6> sections;
  std::vector<fe::ShellPlasticityParentInput> parents;
  static tl::math::Vec3 Position(std::size_t n) {
    if(n>=1028) return {514.,double(n-1028),0.};
    const auto corner=n%4;
    return {2.*double(n/4)+double(corner==1||corner==2),double(corner>=2),0.};
  }
  static std::uint64_t NodeId(std::size_t n) {
    return n<1028?1000+n:(std::uint64_t{1}<<54)+n;
  }
  template<class Input,std::size_t N>
  void Set(Input& input,const std::array<std::size_t,N>& nodes,std::size_t part) {
    input.density=materials[part].density_kg_m3;
    input.young_modulus=materials[part].young_pa;
    input.poisson_ratio=materials[part].poisson_ratio;
    input.thickness=sections[part].thickness_m;
    for(std::size_t n=0;n<N;++n) {
      input.node_ids[n]=static_cast<decltype(input.node_ids[n]+0)>(NodeId(nodes[n]));
      input.position[n]=Position(nodes[n]);
    }
  }
  Fixture() {
    for(std::size_t i=0;i<x0.size();++i) { x0[i]=i/100.; y0[i]=270e6+i*1e6; }
    for(std::size_t i=0;i<x1.size();++i) { x1[i]=i/100.; y1[i]=180e6+i*1e6; }
    curves={{{270,{x0.data(),y0.data(),46}},{180,{x1.data(),y1.data(),17}}}};
    constexpr std::size_t nq[]{71,126,481,88,18,20},nt[]{2,12,67,6,15,9};
    constexpr double thickness[]{.000731,.003845,.000889,.001648,.00235,.00235};
    std::size_t qi=0,ti=0;
    for(std::size_t part=0;part<6;++part) {
      materials[part]={100+part,part==2?180u:270u,200e9,.3,7890,{}};
      sections[part]={200+part,thickness[part],3};
      for(std::size_t e=0;e<nq[part];++e,++qi) {
        auto& p=q[qi]; const auto n=4*(qi%257);
        p.nodes={n,n+1,n+2,n+3}; p.source_parent_id=10000+qi;
        Set(p.reference,p.nodes,part);
        parents.push_back({fe::ShellBindingFamily::Qeph,qi,p.source_parent_id,
                           300+part,100+part,200+part});
      }
      for(std::size_t e=0;e<nt[part];++e,++ti) {
        auto& p=t[ti]; const auto n=4*(ti%257);
        p.nodes=ti+1==TCount?std::array<std::size_t,3>{1025,1028,1029}:
                              std::array<std::size_t,3>{n,n+1,n+2};
        p.source_parent_id=20000+ti; Set(p.reference,p.nodes,part);
        parents.push_back({fe::ShellBindingFamily::T3,ti,p.source_parent_id,
                           300+part,100+part,200+part});
      }
    }
  }
  fe::ShellBatchCollectionInput geometry() const { return {q.data(),t.data(),q.size(),t.size(),NodeCount}; }
  fe::ShellBatchPlasticityBindingInput catalog() const {
    return {curves.data(),materials.data(),sections.data(),parents.data(),2,6,6,parents.size()};
  }
};
} // namespace host_shell_test
