// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../radioss_type25_runtime/FullLedgerFixture.h"
#include "lib_src/collision/radioss_type25/activity_operands/State.h"
#include "lib_src/collision/radioss_type25/normal_activation/Values.h"
#include "NativeOracle.h"
#include <array>
namespace activity_operands_test {
namespace n=tlfea::contact::radioss_type25; namespace a=n::activity_operands;
namespace fe=tl::fea; namespace native=type25_activity_native;
inline bool Good(cudaError_t e) { EXPECT_EQ(e,cudaSuccess)<<cudaGetErrorString(e); return e==cudaSuccess; }
inline bool Good(n::TransactionReport r) { EXPECT_EQ(r.status,n::TransactionStatus::Ok)<<r.message<<" row="<<r.row;return r.status==n::TransactionStatus::Ok; }
template<class T> std::vector<T> Read(const T* data,std::size_t count,cudaStream_t stream) {
  std::vector<T> out(count);if(count)EXPECT_TRUE(Good(cudaMemcpyAsync(out.data(),data,count*sizeof(T),cudaMemcpyDeviceToHost,stream)));
  EXPECT_TRUE(Good(cudaStreamSynchronize(stream)));return out;
}
struct Resources {
  cudaStream_t stream=nullptr;void* arena=nullptr;
  ~Resources(){if(arena)cudaFree(arena);if(stream)cudaStreamDestroy(stream);}
};
// Numerical operand coupon. Real physical-owner receipt authentication is
// separately exercised by the owning snapshot and transaction integration gates.
struct GpuFixture {
  type25_source_test::FullLedgerFixture physical;
  n::ContactSourceInput source;
  n::activity_source::Plan plan;
  n::current_normals::Topology topology;
  n::activity_source::Controls controls;
  n::UnitScale units{.001,1000,1};
  bool normals=true;
  Resources resources;
  a::BorrowedSlot borrowed;
  std::uint8_t *qbase=nullptr,*qcurrent=nullptr,*tbase=nullptr,*tcurrent=nullptr;
  a::State operands;
  explicit GpuFixture(bool moving=true,bool deletion=true):controls{
      deletion?n::activity_source::Deletion::ContainingElement:n::activity_source::Deletion::Disabled,
      false,n::startup::SolidErosion::Disabled},normals(moving) {
    source=physical.Contact();
    const auto report=plan.Initialize({physical.physical,nullptr},source,controls);
    if(!Good(report))throw std::runtime_error(report.message);
    const auto& s=physical.starter;
    topology={s.mains,s.node_count,s.primary_count,s.main_count,s.starter.reference_count,
      {s.normal_offsets,s.starter.reference_count+1,s.normal_mains,s.normal_incidence_count}};
  }
  a::BorrowedSlot Shape() const {
    a::BorrowedSlot out;out.main_capacity=source.selection.main_count;
    out.normal_capacity=out.free_capacity=normals?out.main_capacity:0;
    out.primary_capacity=source.primary_main_count;out.secondary_capacity=source.selection.secondary_count;
    if(normals)for(const auto& main:physical.mains)out.initial_free_count+=n::normal_activation::detail::FreeMain(main);
    return out;
  }
  bool Initialize() {
    const auto& s=source.selection;borrowed=Shape();
    if(!Good(cudaStreamCreate(&resources.stream)))return false;
    tl::util::BoundedArenaLayout layout(1u<<20);tl::util::ArenaRegion m,norm,coef,free,ms,ss,qb,qc,tb,tc;
    if(!layout.Append<n::lifecycle::Main>(s.main_count,m)||
       !layout.Append<n::startup::Main>(normals?s.main_count:0,norm)||
       !layout.Append<double>(normals?s.main_count:0,coef)||
       !layout.Append<std::uint32_t>(normals?s.main_count:0,free)||
       !layout.Append<double>(source.primary_main_count,ms)||!layout.Append<double>(s.secondary_count,ss)||
       !layout.Append<std::uint8_t>(2,qb)||!layout.Append<std::uint8_t>(2,qc)||
       !layout.Append<std::uint8_t>(1,tb)||!layout.Append<std::uint8_t>(1,tc))return false;
    if(!Good(cudaMalloc(&resources.arena,layout.bytes())))return false;
    auto* base=resources.arena;
    borrowed.mains=tl::util::ArenaPointer<n::lifecycle::Main>(base,m);
    borrowed.main_stiffness_si=tl::util::ArenaPointer<double>(base,ms);
    borrowed.secondary_stiffness_si=tl::util::ArenaPointer<double>(base,ss);
    if(normals){borrowed.normal_mains=tl::util::ArenaPointer<n::startup::Main>(base,norm);
      borrowed.normal_coefficients=tl::util::ArenaPointer<double>(base,coef);
      borrowed.free_mains=tl::util::ArenaPointer<std::uint32_t>(base,free);}
    qbase=tl::util::ArenaPointer<std::uint8_t>(base,qb);qcurrent=tl::util::ArenaPointer<std::uint8_t>(base,qc);
    tbase=tl::util::ArenaPointer<std::uint8_t>(base,tb);tcurrent=tl::util::ArenaPointer<std::uint8_t>(base,tc);
    std::vector<double> coefficients(s.main_count),main_si(source.primary_main_count),secondary_si(s.secondary_count);
    std::vector<std::uint32_t> free_ids;n::units_detail::Factors factor;n::units_detail::Make(units,factor);
    for(std::size_t i=0;i<s.main_count;++i){coefficients[i]=s.mains[i].coefficient;
      if(i<main_si.size())main_si[i]=coefficients[i]*factor.stiffness;
      if(normals&&n::normal_activation::detail::FreeMain(s.mains[i]))free_ids.push_back(std::uint32_t(i+1));}
    for(std::size_t i=0;i<s.secondary_count;++i)secondary_si[i]=s.secondary[i].coefficient*factor.stiffness;
    const auto copy=[&](void* to,const void* from,std::size_t bytes){return !bytes||Good(cudaMemcpyAsync(to,from,bytes,cudaMemcpyHostToDevice,resources.stream));};
    if(!copy(borrowed.mains,s.mains,m.bytes)||!copy(borrowed.main_stiffness_si,main_si.data(),ms.bytes)||
       !copy(borrowed.secondary_stiffness_si,secondary_si.data(),ss.bytes)||
       (normals&&(!copy(borrowed.normal_mains,topology.mains,norm.bytes)||
        !copy(borrowed.normal_coefficients,coefficients.data(),coef.bytes)||
        !copy(borrowed.free_mains,free_ids.data(),free_ids.size()*sizeof(std::uint32_t))))||
       !Good(cudaStreamSynchronize(resources.stream)))return false;
    return Good(operands.Initialize(plan,source,normals?&topology:nullptr,units,borrowed,resources.stream));
  }
  fe::PhysicalActivityDeviceView Activity(std::array<std::uint8_t,2> before,
      std::array<std::uint8_t,2> after,std::uint8_t t_before=1,std::uint8_t t_after=1,std::uint64_t generation=1) {
    EXPECT_TRUE(Good(cudaMemcpyAsync(qbase,before.data(),2,cudaMemcpyHostToDevice,resources.stream)));
    EXPECT_TRUE(Good(cudaMemcpyAsync(qcurrent,after.data(),2,cudaMemcpyHostToDevice,resources.stream)));
    EXPECT_TRUE(Good(cudaMemcpyAsync(tbase,&t_before,1,cudaMemcpyHostToDevice,resources.stream)));
    EXPECT_TRUE(Good(cudaMemcpyAsync(tcurrent,&t_after,1,cudaMemcpyHostToDevice,resources.stream)));
    EXPECT_TRUE(Good(cudaStreamSynchronize(resources.stream)));
    fe::PhysicalActivityDeviceView v;v.qeph={qbase,qcurrent,{2}};v.t3={tbase,tcurrent,{1}};
    v.accepted.owner_id=77;v.accepted.node_count=source.selection.node_count;
    v.stream=resources.stream;v.generation=generation;v.attempt=generation;return v;
  }
};
} // namespace activity_operands_test
