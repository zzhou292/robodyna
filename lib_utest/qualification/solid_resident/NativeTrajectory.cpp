// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeTrajectory.h"
#include "NativeChecks.h"
#include "HephDiagnostic.h"
#include "lib_src/elements/solids/resident/Traits18.h"
#include "lib_src/elements/solids/resident/Traits24.h"
#include "lib_src/elements/solids/resident/Traits6z.h"

namespace solid_resident_test {
struct NativeTrajectory::Impl {
  explicit Impl(const s::Model& input):model(input) {}
  s::Model model;
  solid18_force_test::NativeState a;
  heph_test::NativeHistory b;
  solid6z_force_test::NativeHistory c;
  solid18_force_test::NativeResult next_a;
  heph_test::NativeTrial next_b;
  solid6z_force_test::NativeResult next_c;
  tl::fea::solid24::ForceTrial host_b;
  tl::fea::solid24::History accepted_host_b;
  tl::fea::solid24::PrescribedInterval interval_b;
};
NativeTrajectory::NativeTrajectory()=default;
NativeTrajectory::~NativeTrajectory()=default;
template<class Traits> typename Traits::Interval Packet(const typename Traits::Parent& p,
    const std::vector<double>& x,const std::vector<double>& v,double time,double dt,std::uint64_t epoch) {
  auto interval=Traits::Phase(time,dt,epoch);
  for (unsigned n=0;n<Traits::nodes;++n) {
    const auto i=3*p.domain_nodes[n];
    Traits::Node(interval,n,{x[i],x[i+1],x[i+2]},{v[i],v[i+1],v[i+2]});
  }
  return interval;
}
bool NativeTrajectory::Initialize(const s::Model& model,tl::math::Vec3 velocity) {
  impl_=std::make_unique<Impl>(model);
  auto& p=*impl_;
  if (model.solid18().size()!=1 || model.solid24().size()!=1 || model.solid6z().size()!=1) return false;
  p.a=solid18_force_test::NativeInitial(model.solid18()[0].reference.input());
  p.b=heph_test::InitializeNative(model.solid24()[0].reference.input());
  if (!p.c.Initialize(model.solid6z()[0].reference.input(),
      model.materials42()[model.solid6z()[0].material_index].value)) return false;
  std::vector<double> x,v;
  for (const auto& node:model.domain()->nodes()) {
    Vector(x,node.position);
    Vector(v,velocity);
  }
  using namespace s::batch_detail;
  p.next_a=solid18_force_test::Native(model.materials36()[model.solid18()[0].material_index].value,p.a,
      Packet<Traits18>(model.solid18()[0],x,v,0,0,0));
  p.next_b=heph_test::NativeStep(p.b,Packet<Traits24>(model.solid24()[0],x,v,0,0,0),
      model.materials42()[model.solid24()[0].material_index].value);
  p.next_c=p.c.InitializeForce(velocity);
  if (HephDiagnosticEnabled()) {
    const auto& parent=model.solid24()[0];
    const auto& material=model.materials42()[parent.material_index].value;
    const auto status=tl::fea::solid24::InitializeForce(parent.reference,material,velocity,p.host_b);
    EXPECT_EQ(status,tl::fea::solid24::ForceStatus::Success);
  }
  return !p.next_a.status&&!p.next_b.status&&!p.next_c.status;
}
bool NativeTrajectory::Evaluate(const std::vector<double>& x,const std::vector<double>& v,
    double time,double dt,std::uint64_t epoch) {
  auto& p=*impl_;
  const auto& model=p.model;
  if (x.size()!=3*model.domain()->node_count() || v.size()!=x.size()) return false;
  using namespace s::batch_detail;
  p.next_a=solid18_force_test::Native(model.materials36()[model.solid18()[0].material_index].value,p.a,
      Packet<Traits18>(model.solid18()[0],x,v,time,dt,epoch));
  p.next_b=heph_test::NativeStep(p.b,Packet<Traits24>(model.solid24()[0],x,v,time,dt,epoch),
      model.materials42()[model.solid24()[0].material_index].value);
  p.next_c=p.c.Evaluate(Packet<Traits6z>(model.solid6z()[0],x,v,time,dt,epoch));
  if (HephDiagnosticEnabled()) {
    const auto& parent=model.solid24()[0];
    p.interval_b=Packet<Traits24>(parent,x,v,time,dt,epoch);
    const auto status=tl::fea::solid24::EvaluateForce(parent.reference,p.accepted_host_b,p.interval_b,
        model.materials42()[parent.material_index].value,p.host_b);
    EXPECT_EQ(status,tl::fea::solid24::ForceStatus::Success);
  }
  return !p.next_a.status&&!p.next_b.status&&!p.next_c.status;
}
void NativeTrajectory::Compare(const Results& results,double time,std::uint64_t epoch) const {
  if (HephDiagnosticEnabled())
    HephDiagnostic(impl_->interval_b,impl_->accepted_host_b,impl_->host_b,results.b,impl_->next_b,epoch);
  CheckNative(results.a,impl_->next_a);
  CheckNative(results.b,impl_->next_b);
  CheckNative(results.c,impl_->next_c,impl_->model.solid6z()[0].reference.geometry().volume_m3);
  EXPECT_EQ(results.a.stamp.time_s,time);EXPECT_EQ(results.a.stamp.sample_index,epoch);
  EXPECT_EQ(results.b.stamp.time_s,time);EXPECT_EQ(results.b.stamp.sample_index,epoch);
  EXPECT_EQ(results.c.stamp.time_s,time);EXPECT_EQ(results.c.stamp.sample_index,epoch);
}
void NativeTrajectory::Accept() {
  impl_->a=impl_->next_a.next;
  heph_test::AcceptNative(impl_->next_b,impl_->b);
  impl_->c.Accept(impl_->next_c);
  if (HephDiagnosticEnabled()) impl_->accepted_host_b=impl_->host_b.proposed_history;
}
} // namespace solid_resident_test
