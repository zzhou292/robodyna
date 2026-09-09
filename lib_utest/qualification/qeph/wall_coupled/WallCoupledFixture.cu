#include "WallCoupledFixture.h"
#include "lib_utest/qualification/surface_contact/NodalWallOwnerFixture.h"
#include <sstream>

namespace qeph_wall_test {
bool WallRig::Initialize(double h) {
  auto& r=shell;
  EXPECT_TRUE(r.count==1 || r.count==2); EXPECT_TRUE(h==H0 || h==H0/2);
  if((r.count!=1 && r.count!=2) || (h!=H0 && h!=H0/2)) return false;
  r.h=h;
  r.mass.fill(0); r.inertia.fill(0); r.physical.fill(0); r.added.fill(0);
  const double cosine=std::sqrt(1-Slope*Slope);
  for(unsigned e=0;e<r.count;++e) {
    auto input=r.element[e].reference.input;
    input.young_modulus=Young; input.density=Density; input.thickness=Thickness; input.poisson_ratio=Poisson;
    for(unsigned i=0;i<4;++i) {
      const auto old=input.position[i]; const double u=Side*(old.x-.5*r.count),v=Side*old.y;
      input.position[i]={Preload+Slope*u,cosine*u,v};
      const auto n=r.element[e].nodes[i]; const auto x=input.position[i];
      r.x[3*n]=x.x; r.x[3*n+1]=x.y; r.x[3*n+2]=x.z;
      parents[e].nodes[i]=n;
    }
    parents[e].parent_element_id=1001+e; parents[e].feature_id=2001+e; parents[e].parent_face_id=0;
    const auto status=q::InitializeReference(input,r.element[e].reference);
    EXPECT_EQ(status,q::Status::kSuccess); if(status!=q::Status::kSuccess) return false;
    for(unsigned i=0;i<4;++i) {
      const auto n=r.element[e].nodes[i]; const auto& ref=r.element[e].reference;
      r.mass[n]+=ref.nodal_mass[i]; r.inertia[n]+=ref.isotropic_inertia[i];
      r.physical[n]+=ref.physical_inertia[i]; r.added[n]+=ref.added_inertia[i];
    }
  }
  for(unsigned n=0;n<r.n;++n) { r.inverse[n]=1/r.mass[n]; r.inverse_j[n]=1/r.inertia[n]; }
  if(!r.InitializeOwner()) return false;
  auto config=r.Config(q::BatchUsage::CoupledForces);
  config.qualification_id=WallQualification; config.configuration_id=ShellConfiguration;
  auto b=r.batch.Initialize(config,r.element.data());
  EXPECT_EQ(b.status,q::BatchStatus::Success)<<b.message;
  if(b.status!=q::BatchStatus::Success || !r.Bind()) return false;
  const sc::VectorView positions{r.x.data(),r.n,3,1};
  const auto prepared=reference.Initialize(positions,parents.data(),r.count);
  EXPECT_EQ(prepared.status,sc::Q4ParametricStatus::Ok)<<prepared.message;
  if(prepared.status!=sc::Q4ParametricStatus::Ok) return false;
  const sc::NodalWallParentInput inputs[]{{&reference,0,nullptr},{&reference,1,nullptr}};
  auto w=weights.Initialize(r.n,inputs,r.count); EXPECT_EQ(w.status,sc::NodalWallStatus::Ok);
  if(w.status!=sc::NodalWallStatus::Ok) return false;
  sc::NodalWallDeviceConfig c; c.owner=r.owner.accepted(); c.configuration_id=WallConfiguration;
  c.qualification_id=WallQualification; c.wall_binding_id=WallBinding; c.law=Law();
  const auto result=wall.Initialize(c,mesh.view(),weights,positions,r.inverse.data(),r.fixed.data(),motion);
  EXPECT_EQ(result.status,sc::NodalWallDeviceStatus::Ok)<<result.message;
  return result.status==sc::NodalWallDeviceStatus::Ok;
}
sc::NodalWallResult WallRig::Host(const Snapshot& s,std::uint64_t epoch,std::uint64_t attempt) const {
  sc::NodalWallResult result;
  const auto& r=shell;
  const auto report=sc::EvaluateNodalWallContact(weights,{s.x.data(),r.n,3,1},{s.v.data(),r.n,3,1},
    {r.inverse.data(),r.fixed.data(),r.n,epoch,sc::TranslationMassModel::kIsotropicLumped},Law(),attempt,&result);
  EXPECT_EQ(report.status,sc::NodalWallStatus::Ok); return result;
}
Loads ContactLoads(const sc::NodalWallResult& result) {
  Loads load;
  for(unsigned j=0;j<result.node_count;++j) {
    const auto& point=result.nodes[j]; const auto i=3*point.node;
    load.force[i]=point.force_world.x; load.force[i+1]=point.force_world.y; load.force[i+2]=point.force_world.z;
  }
  return load;
}
void ContactAgreement(const sc::NodalWallDeviceResults& result,const sc::NodalWallResult& host) {
  nodal_wall_owner_test::Same(result,host); // Existing complete independent host/device field comparison.
  for(unsigned i=0;i<result.diagnostics.node_count;++i) EXPECT_NE(result.wall_face[i],0u);
}
void Property(const std::string& name,double value) {
  std::ostringstream text; text<<std::setprecision(std::numeric_limits<double>::max_digits10)<<value;
  ::testing::Test::RecordProperty(name,text.str());
}
} // namespace qeph_wall_test
