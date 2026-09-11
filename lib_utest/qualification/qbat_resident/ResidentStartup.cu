// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"
#include "lib_src/elements/type25/Type25Model.h"

namespace qbat_resident_test {
Rig::~Rig() { if(loads) cudaFree(loads); }
bool Rig::Initialize(const fe::ShellFormulationScope& scope,bool use_coupled,bool moving,double h,const fe::type25::Model* connector_model) {
  int devices=0;
  EXPECT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
  EXPECT_GT(devices,0);
  if(devices<=0) return false;
  binding=scope.binding;
  mass=scope.mass;
  if(bool(mass)!=bool(connector_model)) return false;
  coupled=use_coupled;
  dt=h;
  const auto count=binding->node_count();
  Fields initial(count);
  std::vector<double> inverse(count),inverse_inertia(count);
  std::vector<std::uint8_t> fixed(count),rotation_fixed(count);
  for(std::size_t n=0;n<count;++n) {
    const auto& node=binding->nodes()[n];
    initial.x[3*n]=node.position.x;
    initial.x[3*n+1]=node.position.y;
    initial.x[3*n+2]=node.position.z;
    initial.v[3*n]=moving?-8.:0.;
    initial.q[4*n]=1;
    inverse[n]=1/(scope.mass?scope.mass->nodes()[n].coefficients.mass:node.native.mass);
    inverse_inertia[n]=1/(scope.mass?scope.mass->nodes()[n].coefficients.isotropic_inertia:node.native.isotropic_inertia);
  }
  fe::NodalStateConfig config;
  config.node_count=count;
  config.max_nodes=std::max<std::size_t>(2048,count);
  config.max_device_bytes=32*1024*1024;
  config.fixed_dt=dt;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto initialized=owner.Initialize(config,{initial.x.data(),initial.v.data(),initial.omega.data(),count,initial.q.data()},
      inverse.data(),fe::NodalDofConfig{fixed.data(),rotation_fixed.data(),inverse_inertia.data()});
  EXPECT_EQ(initialized.status,fe::NodalStatus::Ok)<<initialized.message;
  if(initialized.status!=fe::NodalStatus::Ok) return false;
  fe::ShellBatchStartup startup;
  if(moving) startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,{-8,0,0}};
  const auto resident=fe::ShellResidentLimits::Vehicle();
  if(binding->qeph_count()) {
    fe::qeph::QephBatchConfig q;
    q.owner=owner.accepted();q.configuration_id=Configuration;q.qualification_id=Qualification;
    q.element_count=binding->qeph_count();q.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;
    q.storage_limits=resident;q.startup=startup;
    q.usage=coupled?fe::qeph::BatchUsage::CoupledForces:fe::qeph::BatchUsage::PrescribedFields;
    const auto result=qeph.InitializeFormulations(q,scope,fe::ShellBatchFailureLimits::Vehicle());
    EXPECT_EQ(result.status,fe::qeph::BatchStatus::Success)<<result.message;
    if(result.status!=fe::qeph::BatchStatus::Success) return false;
  }
  if(binding->t3_count()) {
    fe::t3::T3BatchConfig t;
    t.owner=owner.accepted();t.configuration_id=Configuration;t.qualification_id=Qualification;
    t.element_count=binding->t3_count();t.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;
    t.storage_limits=resident;t.startup=startup;
    t.usage=coupled?fe::t3::BatchUsage::CoupledForces:fe::t3::BatchUsage::PrescribedFields;
    const auto result=t3.InitializeFormulations(t,scope,fe::ShellBatchFailureLimits::Vehicle());
    EXPECT_EQ(result.status,fe::t3::BatchStatus::Success)<<result.message;
    if(result.status!=fe::t3::BatchStatus::Success) return false;
  }
  qb::BatchConfig b;
  b.owner=owner.accepted();b.configuration_id=Configuration;b.qualification_id=Qualification;
  b.element_count=binding->qbat_count();b.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;
  b.storage_limits=resident;b.startup=startup;
  b.usage=coupled?qb::BatchUsage::CoupledForces:qb::BatchUsage::PrescribedFields;
  const auto built=qbat.InitializeFormulations(b,scope);
  EXPECT_EQ(built.status,qb::BatchStatus::Success)<<built.message;
  if(built.status!=qb::BatchStatus::Success) return false;
  if(connector_model) {
    fe::type25::BatchConfig c;
    c.owner=owner.accepted();
    c.configuration_id=Configuration;
    c.qualification_id=Qualification;
    c.element_count=connector_model->connection_count();
    c.startup=startup;
    const auto result=connector.InitializeJoined(c,*connector_model,*mass);
    EXPECT_EQ(result.status,fe::type25::BatchStatus::Success)<<result.message;
    if(result.status!=fe::type25::BatchStatus::Success) return false;
  }
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  const auto begun=owner.BeginTrial(&token,&view);
  EXPECT_EQ(begun.status,fe::NodalStatus::Ok);
  if(begun.status!=fe::NodalStatus::Ok||!Assemble(view)) return false;
  Discard();
  const fe::ShellFormulationParticipants participants{
      binding->qeph_count()?&qeph:nullptr,binding->t3_count()?&t3:nullptr,&qbat,mass?&connector:nullptr};
  const auto attached=publication.InitializeFormulations(owner,participants,fe::ShellPublicationLimits::Vehicle());
  EXPECT_EQ(attached.status,fe::ShellPublicationStatus::Success)<<attached.message;
  if(attached.status!=fe::ShellPublicationStatus::Success) return false;
  return cudaMalloc(reinterpret_cast<void**>(&loads),6*count*sizeof(double))==cudaSuccess;
}
bool Rig::Assemble(fe::NodalAssemblyView view) {
  if(binding->qeph_count()) {
    const auto result=qeph.AssembleAccepted(owner,view);
    EXPECT_EQ(result.status,fe::qeph::BatchStatus::Success)<<result.message;
    if(result.status!=fe::qeph::BatchStatus::Success) return false;
  }
  if(binding->t3_count()) {
    const auto result=t3.AssembleAccepted(owner,view);
    EXPECT_EQ(result.status,fe::t3::BatchStatus::Success)<<result.message;
    if(result.status!=fe::t3::BatchStatus::Success) return false;
  }
  const auto result=qbat.AssembleAccepted(owner,view);
  EXPECT_EQ(result.status,qb::BatchStatus::Success)<<result.message;
  if(result.status!=qb::BatchStatus::Success) return false;
  if(mass) {
    const auto assembled=connector.AssembleAccepted(owner,view);
    EXPECT_EQ(assembled.status,fe::type25::BatchStatus::Success)<<assembled.message;
    if(assembled.status!=fe::type25::BatchStatus::Success) return false;
  }
  return true;
}
void Rig::Discard() {
  owner.Discard();
  publication.DiscardTrial();
  qeph.DiscardTrial();
  t3.DiscardTrial();
  qbat.DiscardTrial();
  connector.DiscardTrial();
}
} // namespace qbat_resident_test
