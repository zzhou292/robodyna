// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "FrozenFormulation.h"
#include "FrozenExecution.h"
#include "lib_src/elements/ShellFormulationOutputRanges.h"
#include "lib_src/elements/ShellExecutionOutputRanges.h"
#include "lib_src/elements/ShellCatalogCurveView.h"
#include <array>
#include <cstdint>
namespace shell_output_range_test {
struct Probe {const void* pointer;std::size_t bytes;};
inline const void* Pointer(std::uintptr_t value) {return reinterpret_cast<const void*>(value);}
inline std::uintptr_t Address(const void* value) {return reinterpret_cast<std::uintptr_t>(value);}
// Numeric address tokens only; no probe dereferences or writes output memory.
// Boundaries include exact starts/ends, empty ranges and one-byte crossings.
inline std::vector<Probe> Around(const void* pointer,std::size_t bytes) {
  if(!bytes)return {{pointer,0},{pointer,1}};
  const auto first=Address(pointer),end=first+bytes;
  std::vector<Probe> out;
  for(auto position:{first,first+bytes/2,end-1,end})for(std::size_t width:{0u,1u,2u})out.push_back({Pointer(position),width});
  out.push_back({Pointer(first-1),1});out.push_back({Pointer(first-1),2});
  out.push_back({pointer,bytes});out.push_back({pointer,bytes+1});
  out.push_back({Pointer(first-1),bytes+2});return out;
}
inline std::vector<Probe> Extreme() {
  return {{nullptr,0},{nullptr,1},{Pointer(1),0},{Pointer(1),SIZE_MAX},
      {Pointer(UINTPTR_MAX),0},{Pointer(UINTPTR_MAX),1},{Pointer(UINTPTR_MAX-7),7},
      {Pointer(UINTPTR_MAX-7),8},{Pointer(UINTPTR_MAX-7),SIZE_MAX}};
}
template<class Check> void BindingProbes(const fe::ShellBatchBinding& binding,Check check) {
  for(auto p:Around(&binding,sizeof(binding)))check(p);
  for(auto p:Around(binding.nodes().data(),binding.node_count()*sizeof(fe::ShellBindingNode)))check(p);
  const auto& words=binding.inventory().words();
  for(auto p:Around(words.data(),words.size()*sizeof(std::uint64_t)))check(p);
  for(std::size_t i=0;i<binding.qeph_count();++i) {
    for(auto p:Around(&binding.qeph_reference(i),sizeof(fe::qeph::ReferenceData)))check(p);
    for(auto p:Around(binding.qeph_nodes(i).data(),sizeof(std::array<std::size_t,4>)))check(p);
  }
  for(std::size_t i=0;i<binding.t3_count();++i) {
    for(auto p:Around(&binding.t3_reference(i),sizeof(fe::t3::ReferenceData)))check(p);
    for(auto p:Around(binding.t3_nodes(i).data(),sizeof(std::array<std::size_t,3>)))check(p);
  }
  for(std::size_t i=0;i<binding.qbat_count();++i) {
    for(auto p:Around(&binding.qbat_reference(i),sizeof(fe::qbat::Reference)))check(p);
    for(auto p:Around(binding.qbat_nodes(i).data(),sizeof(std::array<std::size_t,4>)))check(p);
  }
}
inline void SameBinding(const fe::ShellBatchBinding& binding,Probe p) {
  EXPECT_EQ(fe::shell_formulation_detail::BindingOutputDisjoint(binding,p.pointer,p.bytes),
      fe::shell_formulation_detail::frozen341::BindingOutputDisjoint(binding,p.pointer,p.bytes))<<Address(p.pointer)<<" bytes="<<p.bytes;
}
inline void SameScope(const fe::ShellFormulationScope& scope,Probe p) {
  EXPECT_EQ(fe::shell_formulation_detail::OutputDisjoint(scope,p.pointer,p.bytes),
      fe::shell_formulation_detail::frozen341::OutputDisjoint(scope,p.pointer,p.bytes))<<Address(p.pointer)<<" bytes="<<p.bytes;
}
inline void SameExecution(const fe::ShellExecutionBinding& execution,Probe p) {
  EXPECT_EQ(fe::shell_execution_detail::OutputDisjoint(execution,p.pointer,p.bytes),
      fe::shell_execution_detail::frozen341::OutputDisjoint(execution,p.pointer,p.bytes))<<Address(p.pointer)<<" bytes="<<p.bytes;
}
template<class Check> void CatalogProbes(const Catalog& catalog,Check check) {
  for(auto p:Around(&catalog,sizeof(catalog)))check(p);
  const fe::shell_batch_plasticity_detail::CatalogCurveView curves(catalog);
  if(curves.count)for(const void* values:{curves.x,curves.y})for(auto p:Around(values,curves.count*sizeof(double)))check(p);
  const auto& words=catalog.inventory().words();
  for(auto p:Around(words.data(),words.size()*sizeof(std::uint64_t)))check(p);
  for(std::size_t i=0;i<catalog.parent_count();++i)for(auto p:Around(catalog.parent(i),sizeof(fe::ShellPlasticityParentInput)))check(p);
}
}
