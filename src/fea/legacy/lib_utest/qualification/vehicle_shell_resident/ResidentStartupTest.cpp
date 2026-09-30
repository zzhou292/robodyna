#include "../vehicle_shell_host/VehicleShellFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <cstring>
namespace vehicle_resident_test {
namespace fe=tl::fea;namespace q=fe::qeph;namespace t=fe::t3;
template<class Config> Config ConfigFor(std::size_t nodes,std::size_t parents,bool vehicle) {
  Config c;c.element_count=parents;c.owner.owner_id=123;c.owner.node_count=nodes;c.owner.fixed_dt=1./1048576;
  c.owner.has_rotations=true;c.owner.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  c.owner.velocity_phase=fe::NodalVelocityPhase::Collocated;c.configuration_id=17;c.qualification_id=19;
  c.usage=decltype(c.usage)::PrescribedFields;c.storage_limits=vehicle?fe::ShellResidentLimits::Vehicle():fe::ShellResidentLimits{1024,2048,8*1024*1024};
  c.max_device_bytes=vehicle?fe::MaxVehicleShellResidentDeviceBytes:fe::MaxShellResidentDeviceBytes;return c;
}
template<class Layout> struct Arena {
  Layout layout;tl::util::HostArena memory;
  Arena(std::size_t elements,std::size_t nodes) {
    EXPECT_TRUE(layout.Initialize(elements,nodes,fe::MaxVehicleShellResidentDeviceBytes));
    EXPECT_TRUE(memory.Initialize(layout.bytes));
  }
  auto storage(){return layout.Construct(memory);}
};
template<class Storage> void SameModel(const Storage& a,const Storage& b,std::size_t parents,std::size_t nodes) {
  EXPECT_EQ(std::memcmp(a.model.element,b.model.element,parents*sizeof(*a.model.element)),0);
  EXPECT_EQ(std::memcmp(a.slab[0].element,b.slab[0].element,parents*sizeof(*a.slab[0].element)),0);
  EXPECT_EQ(std::memcmp(a.model.initial_position,b.model.initial_position,nodes*sizeof(*a.model.initial_position)),0);
  for(auto pair:{std::pair{a.model.mass,b.model.mass},std::pair{a.model.inertia,b.model.inertia},
      std::pair{a.model.physical,b.model.physical},std::pair{a.model.added,b.model.added}})
    EXPECT_EQ(std::memcmp(pair.first,pair.second,nodes*sizeof(double)),0);
}
TEST(VehicleResidentStartup, IndexedAndLegacyNativePreparationRetainCompleteModelBits) {
  vehicle_shell_test::Fixture f(100,30,129);fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.Initialize(f.input(),fe::ShellHostBindingLimits{}).status,fe::ShellBindingStatus::Success);
  Arena<q::batch_detail::Layout> qa(100,129),qb(100,129);auto* a=qa.storage();auto* b=qb.storage();
  ASSERT_NE(a,nullptr);ASSERT_NE(b,nullptr);
  ASSERT_EQ(q::batch_detail::BuildModel(ConfigFor<q::QephBatchConfig>(129,100,false),nullptr,a->model,a->slab[0],&binding).status,q::BatchStatus::Success);
  ASSERT_EQ(q::batch_detail::BuildModel(ConfigFor<q::QephBatchConfig>(129,100,true),nullptr,b->model,b->slab[0],&binding).status,q::BatchStatus::Success);
  SameModel(*a,*b,100,129);
  Arena<t::batch_detail::Layout> ta(30,129),tb(30,129);auto* x=ta.storage();auto* y=tb.storage();
  ASSERT_NE(x,nullptr);ASSERT_NE(y,nullptr);
  ASSERT_EQ(t::batch_detail::BuildModel(ConfigFor<t::T3BatchConfig>(129,30,false),nullptr,x->model,x->slab[0],&binding).status,t::BatchStatus::Success);
  ASSERT_EQ(t::batch_detail::BuildModel(ConfigFor<t::T3BatchConfig>(129,30,true),nullptr,y->model,y->slab[0],&binding).status,t::BatchStatus::Success);
  SameModel(*x,*y,30,129);
}
TEST(VehicleResidentStartup, NativePreparationCoversBeyondOldBoundsAndFinalGlobalNode) {
  vehicle_shell_test::Fixture f;fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.Initialize(f.input(),fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
  Arena<q::batch_detail::Layout> qa(f.q.size(),f.count);auto* qmodel=qa.storage();ASSERT_NE(qmodel,nullptr);
  const auto qr=q::batch_detail::BuildModel(ConfigFor<q::QephBatchConfig>(f.count,f.q.size(),true),nullptr,qmodel->model,qmodel->slab[0],&binding);
  ASSERT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  Arena<t::batch_detail::Layout> ta(f.t.size(),f.count);auto* tmodel=ta.storage();ASSERT_NE(tmodel,nullptr);
  const auto tr=t::batch_detail::BuildModel(ConfigFor<t::T3BatchConfig>(f.count,f.t.size(),true),nullptr,tmodel->model,tmodel->slab[0],&binding);
  ASSERT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  for(std::size_t n=0;n<f.count;++n) {
    EXPECT_EQ(qmodel->model.mass[n],binding.nodes()[n].native.mass);
    EXPECT_EQ(tmodel->model.inertia[n],binding.nodes()[n].native.isotropic_inertia);
  }
  EXPECT_EQ(tmodel->model.element[f.t.size()-1].nodes[2],f.count-1);
  EXPECT_EQ(qmodel->model.element[f.q.size()-1].reference.input.node_ids[3],f.q.back().reference.node_ids[3]);
}
t::BatchReport RawT(const std::vector<t::T3BatchElement>& input,bool vehicle) {
  Arena<t::batch_detail::Layout> memory(input.size(),129);auto* data=memory.storage();
  return t::batch_detail::BuildModel(ConfigFor<t::T3BatchConfig>(129,input.size(),vehicle),input.data(),data->model,data->slab[0]);
}
TEST(VehicleResidentStartup, LateInvalidIdentityTopologyAndReferenceKeepLegacyFirstFailureAndRetry) {
  vehicle_shell_test::Fixture f(100,30,129);fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.Initialize(f.input(),fe::ShellHostBindingLimits{}).status,fe::ShellBindingStatus::Success);
  std::vector<t::T3BatchElement> original(4);
  for(unsigned e=0;e<4;++e){original[e].reference=binding.t3_reference(e);
    std::copy(binding.t3_nodes(e).begin(),binding.t3_nodes(e).end(),original[e].nodes);}
  for(unsigned fault=0;fault<6;++fault){auto input=original;
    if(fault==0)input[3]=input[0];
    if(fault==1){input[3].nodes[0]=input[0].nodes[0];input[3].nodes[1]=input[0].nodes[0];input[3].nodes[2]=input[0].nodes[1];}
    if(fault==2)input[3].nodes[1]=input[3].nodes[0];
    if(fault==3)input[3].nodes[2]=129;
    if(fault==4){input[3].reference.input.node_ids[0]=input[0].reference.input.node_ids[0];
      ASSERT_EQ(t::InitializeReference(input[3].reference.input,input[3].reference),t::Status::kSuccess);}
    if(fault==5){input[1].reference.area=-1;input[3]=input[0];}
    const auto a=RawT(input,false),b=RawT(input,true);
    EXPECT_NE(a.status,t::BatchStatus::Success);EXPECT_EQ(a.status,b.status)<<fault;
    EXPECT_EQ(a.element,b.element)<<fault;EXPECT_EQ(a.node,b.node)<<fault;EXPECT_STREQ(a.message,b.message)<<fault;
    EXPECT_EQ(a.element_status,b.element_status)<<fault;
  }
  // Clean input reaches the same final complete-coverage check after every
  // failed private build; no caller input is changed by an index or native check.
  const auto a=RawT(original,false),b=RawT(original,true);
  EXPECT_EQ(a.status,t::BatchStatus::InvalidMass);EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.node,b.node);
}
} // namespace vehicle_resident_test
