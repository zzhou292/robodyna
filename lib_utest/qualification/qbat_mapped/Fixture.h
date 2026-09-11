// SPDX-License-Identifier: MIT
#pragma once
#include "../rigid_assembly_owner/Fixture.h"
#include "../qbat_catalog/Fixture.h"
#include "lib_src/elements/qbat/mapped/Startup.h"
#include "lib_src/elements/qbat/mapped/Stiffness.h"
#include "lib_src/elements/ShellPhysicalOutputRanges.h"

namespace qbat_mapped_test {
namespace fe=tl::fea;
namespace qb=fe::qbat;
namespace mapped=qb::mapped;
namespace batch=qb::batch_detail;
using qbat_binding_test::Bits;
using qbat_binding_test::Bytes;
struct Fixture {
  rigid_assembly_owner_test::Fixture mechanics;
  fe::ElementMassContributions point;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::ShellPhysicalBinding physical;
  Fixture() {
    auto& source=mechanics.source;
    const fe::ElementMassSource records[]{{18000,777,mechanics.domain.Find(777),0},
        {18001,55,mechanics.domain.Find(55),.002}};
    EXPECT_TRUE(point.Initialize(mechanics.domain,{1,1000,records,2}));
    EXPECT_TRUE(ledger.InitializeWithSolids({{&mechanics.shells,&mechanics.springs},&point,&mechanics.solids}));
    EXPECT_TRUE(parts.Initialize(mechanics.topology,ledger,{1000,.001}));
    EXPECT_TRUE(rigid.Initialize(parts,&mechanics.plain));
    qbat_catalog_test::Fixture declaration;
    for (unsigned i=0;i<2;++i) {
      declaration.materials[i].curve_id=0;
      declaration.materials[i].hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      declaration.materials[i].linear={10e6,0};
      declaration.materials[i].rate=declaration.materials[2].rate;
    }
    const fe::ShellPlasticityParentInput parents[]{
        {fe::ShellBindingFamily::Qeph,0,100,1000,1000,1000},
        {fe::ShellBindingFamily::Qeph,1,101,1001,1001,1001},
        {fe::ShellBindingFamily::T3,0,102,2000524,2000524,2000524},
        {fe::ShellBindingFamily::Qbat,0,103,2000524,2000524,2000524}};
    const fe::ShellBatchPlasticityBindingInput input{nullptr,declaration.materials.data(),
        declaration.sections.data(),parents,0,3,3,4};
    EXPECT_EQ(catalog.InitializeFormulations(source.shells,input).status,fe::ShellPlasticityBindingStatus::Success);
    fe::ShellFailureParentInput failures[4];
    for (unsigned i=0;i<4;++i) failures[i]=qbat_catalog_test::Failure(parents[i]);
    for (unsigned i=0;i<2;++i) {
      failures[i].policy=fe::ShellFailurePolicy::Tab1AnyPoint;
      failures[i].constant={};
      failures[i].tab1.table={{-1,0,1},1};
    }
    EXPECT_EQ(failure.Initialize(catalog,failures,4).status,fe::ShellPlasticityBindingStatus::Success);
    EXPECT_TRUE(physical.Initialize({&source.shells,&catalog,&failure,nullptr},ledger));
    for (std::size_t node=0;node<mechanics.m.size();++node) {
      mechanics.m[node]=ledger.nodes()[node].coefficients.mass;
      mechanics.j[node]=ledger.nodes()[node].coefficients.isotropic_inertia;
    }
    mechanics.fixed[mechanics.domain.Find(55)]=0;
    mechanics.DependentInverses(true);
  }
  fe::NodalCinWitnessSource Witnesses() const {
    return {&mechanics.cin_model,mechanics.ranges.data(),mechanics.witnesses.data(),
        mechanics.ranges.size(),mechanics.witnesses.size()};
  }
  qb::BatchConfig Config() const {
    qb::BatchConfig config;
    config.owner.owner_id=71;
    config.owner.node_count=mechanics.domain.node_count();
    config.owner.fixed_dt=mechanics.Config().fixed_dt;
    config.owner.has_rotations=true;
    config.owner.has_rotation_presence=true;
    config.owner.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    config.owner.rigid_groups={1,rigid.groups().size(),rigid.members().size(),
        parts.roots().size(),rigid.plain_source_instance_id()};
    config.configuration_id=81;
    config.qualification_id=82;
    config.element_count=physical.shells()->qbat_count();
    config.usage=qb::BatchUsage::CoupledForces;
    return config;
  }
};
} // namespace qbat_mapped_test
