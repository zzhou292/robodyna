#pragma once
#include "../shell_tab1_force/Tab1ForceFixture.h"
#include "lib_src/elements/ShellBatchBinding.h"

namespace placement_force_test {
namespace base=tab1_force_test;
using base::Q;
using base::T;
using base::Interval;
using base::Exact;
using base::ForceValues;
using base::SectionValues;
using Placement=tl::fea::ShellReferencePlacement;
constexpr Placement Planes[]{Placement::Centered,Placement::TopReferencePlane,Placement::BottomReferencePlane};

template<class F> struct Fixture : base::Fixture<F> {
  explicit Fixture(Placement placement,unsigned mask=0) : base::Fixture<F>(mask) {
    auto input=this->reference.input;
    input.thickness=.00228; // Both original offset windshield layers.
    input.placement=placement;
    if(InitializeReference(input,this->reference)!=F::Status::kSuccess ||
        InitializeLayeredTab1History(this->reference,this->material,this->failure,{},this->accepted)!=F::Status::kSuccess)
      throw std::runtime_error("placed glass reference/history");
    this->accepted.section=tab1_test::Seed(mask);
  }
};
inline tl::fea::ShellBatchBindingInput BindingInput(Placement q,Placement t) {
  tl::fea::ShellBatchBindingInput input;
  input.qeph=Fixture<Q>(q).reference.input;
  input.t3=Fixture<T>(t).reference.input;
  input.qeph_nodes={0,1,2,3};
  input.t3_nodes={4,5,6};
  input.node_count=7;
  for(unsigned n=0;n<3;++n) input.t3.node_ids[n]+=1000;
  return input;
}
} // namespace placement_force_test
