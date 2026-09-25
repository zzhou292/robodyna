// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include "lib_src/elements/ShellBatchStartup.h"
#include "lib_src/solvers/NodalCinRuntime.h"
#include <stdexcept>
#include <vector>
namespace nodal_empty_test {
namespace fe=tl::fea;
namespace tied=tl::constraints::tied_shell;
// GTest-free source/owner setup. A caller may replace every ordered Q4/T3 input
// and resolved catalog before Prepare; no contact pairing or source parser is
// hidden here. All coefficients come from the existing shell ledger.
struct Source {
  std::vector<fe::ShellQephBindingInput> quads;
  std::vector<fe::ShellT3BindingInput> triangles;
  std::vector<fe::ShellPlasticityMaterialInput> materials;
  std::vector<fe::ShellPlasticitySectionInput> sections;
  std::vector<fe::ShellPlasticityParentInput> parents;
  std::vector<fe::ShellPlasticityCurveInput> curves;
  std::size_t nodes=0;
  std::uint64_t source_instance_id=771;
  fe::ShellBatchCollectionInput Mesh() const {
    return {quads.empty()?nullptr:quads.data(),triangles.empty()?nullptr:triangles.data(),quads.size(),triangles.size(),nodes};
  }
  fe::ShellBatchPlasticityBindingInput Catalog() const {
    return {curves.empty()?nullptr:curves.data(),materials.data(),sections.data(),parents.data(),
        curves.size(),materials.size(),sections.size(),parents.size()};
  }
};
inline Source SmallSource() {
  Source s;s.nodes=7;s.quads.resize(1);s.triangles.resize(1);
  auto& q=s.quads[0];q.source_parent_id=100;q.nodes={0,1,2,3};
  const tl::math::Vec3 qx[]{{0,0,0},{.04,0,0},{.04,.02,0},{0,.02,0}};
  q.reference.density=2500;q.reference.young_modulus=60e9;q.reference.poisson_ratio=.25;
  q.reference.thickness=.002;
  for(unsigned k=0;k<4;++k){q.reference.position[k]=qx[k];q.reference.node_ids[k]=10+k;}
  auto& t=s.triangles[0];t.source_parent_id=200;t.nodes={4,5,6};
  const tl::math::Vec3 tx[]{{.01,.005,.01},{.03,.005,.01},{.01,.015,.01}};
  t.reference.density=2500;t.reference.young_modulus=60e9;t.reference.poisson_ratio=.25;t.reference.thickness=.002;
  for(unsigned k=0;k<3;++k){t.reference.position[k]=tx[k];t.reference.node_ids[k]=20+k;}
  fe::ShellPlasticityMaterialInput material;
  material.material_id=1;material.young_pa=60e9;material.poisson_ratio=.25;material.density_kg_m3=2500;
  material.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
  material.linear={300e6,1e9};
  material.rate={true,0,1,10000,tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
  s.materials.push_back(material);s.sections.push_back({1,.002,3,fe::ShellSectionFormulation::LayeredNip3});
  s.parents={{fe::ShellBindingFamily::Qeph,0,100,1,1,1},{fe::ShellBindingFamily::T3,0,200,2,1,1}};
  return s;
}
struct Fixture {
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::NodalNodeDomain domain;fe::ShellNodeMap mapping;fe::NodalCoefficientLedger ledger;
  fe::NodalRigidAssemblyBinding rigid;fe::ShellExecutionBinding execution;fe::ShellPhysicalBinding physical;
  tied::TiedCinAttachmentModel cin;
  std::vector<double> positions,velocities,omega,orientation,mass,inertia,inverse_mass,inverse_inertia;
  std::vector<std::uint8_t> fixed,rotation_fixed;
  fe::ShellBatchStartup startup{fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,{-0.,.5,-1.25}};
  static constexpr std::uint64_t Qualification=902;
  double fixed_dt=1e-9;
  static void Require(bool valid,const char* message){if(!valid)throw std::runtime_error(message);}
  void Prepare(const Source& source,std::vector<std::uint8_t> constraints,
      std::vector<std::uint8_t> rotation_constraints,tl::math::Vec3 common={-0.,.5,-1.25}) {
    Require(constraints.size()==source.nodes&&rotation_constraints.size()==source.nodes,"Complete immutable DOF masks required");
    Require(shells.Initialize(source.Mesh()).status==fe::ShellBindingStatus::Success,"Shell reference binding");
    Require(catalog.InitializeExecutionCatalog(shells,source.Catalog()).status==fe::ShellPlasticityBindingStatus::Success,"Execution catalog");
    std::vector<fe::ShellFailureParentInput> failures;
    for(auto parent:source.parents){fe::ShellFailureParentInput row;row.source=parent;row.policy=fe::ShellFailurePolicy::None;failures.push_back(row);}
    Require(failure.InitializeExecution(catalog,failures.data(),failures.size()).status==fe::ShellPlasticityBindingStatus::Success,"Complete all-None failure scope");
    std::vector<fe::NodalDomainNode> nodes;
    for(std::size_t i=0;i<shells.node_count();++i)nodes.push_back({shells.nodes()[i].source_id,shells.nodes()[i].position});
    Require(bool(domain.Initialize({source.source_instance_id,nodes.data(),nodes.size()})),"Complete physical domain");
    Require(bool(mapping.Initialize(shells,domain)),"Shell/domain mapping");
    Require(bool(ledger.Initialize({&mapping,nullptr,nullptr})),"Physical shell M/J ledger");
    Require(bool(rigid.InitializeEmpty(ledger)),"Explicit empty rigid binding");
    Require(execution.Initialize(catalog,ledger,rigid).status==fe::ShellPlasticityBindingStatus::Success,"Execution roles");
    Require(bool(physical.InitializeExecution({&shells,&catalog,&failure,nullptr},ledger,execution)),"Physical execution binding");
    Require(bool(tied::PrepareEmptyCinAttachments(domain,&cin)),"Explicit empty CIN binding");
    fixed=std::move(constraints);rotation_fixed=std::move(rotation_constraints);startup.uniform_velocity=common;
    const auto n=nodes.size();positions.resize(3*n);velocities.resize(3*n);omega.resize(3*n);orientation.resize(4*n);
    mass.resize(n);inertia.resize(n);inverse_mass.resize(n);inverse_inertia.resize(n);
    for(std::size_t i=0;i<n;++i) {
      const auto& c=ledger.nodes()[i].coefficients;mass[i]=c.mass;inertia[i]=c.isotropic_inertia;
      Require(fixed[i]<=7&&rotation_fixed[i]<=1&&mass[i]>0&&inertia[i]>0,"Finite native shell DOFs");
      inverse_mass[i]=fixed[i]==7?0:1/mass[i];inverse_inertia[i]=rotation_fixed[i]?0:1/inertia[i];
      const auto x=nodes[i].position;positions[3*i]=x.x;positions[3*i+1]=x.y;positions[3*i+2]=x.z;
      const auto v=fe::shell_startup_detail::ProjectVelocity(common,fixed[i]);
      velocities[3*i]=v.x;velocities[3*i+1]=v.y;velocities[3*i+2]=v.z;orientation[4*i]=1;
    }
  }
  void Small(){Prepare(SmallSource(),{7,7,7,7,0,1,2},{1,1,1,1,0,1,0});}
  fe::NodalStateConfig Config() const {
    fe::NodalStateConfig c;c.node_count=mass.size();c.fixed_dt=fixed_dt;c.minimum_dt=1e-12;
    c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;return c;
  }
  fe::HostNodalKinematicsView Kinematics() const{return {positions.data(),velocities.data(),omega.data(),mass.size(),orientation.data()};}
  fe::NodalDofConfig Dofs() const{return {fixed.data(),rotation_fixed.data(),inverse_inertia.data()};}
  fe::NodalCinStartup Cin() const{return {&cin,mass.data(),inertia.data(),nullptr,nullptr,0,Qualification};}
  fe::NodalCinWitnessSource Witnesses() const{return {&cin,nullptr,nullptr,0,0};}
  fe::NodalReport Initialize(fe::FENodalState& owner) const {
    const auto scope=Cin();return owner.Initialize(Config(),Kinematics(),inverse_mass.data(),Dofs(),rigid,&scope);
  }
};
} // namespace nodal_empty_test
