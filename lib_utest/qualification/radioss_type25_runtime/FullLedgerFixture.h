// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <gtest/gtest.h>
#include "SourceAdmissionFixture.h"
#include "../nodal_coefficients/SolidFixture.h"
#include "../qbat_catalog/Fixture.h"

namespace type25_source_test {
// Existing qualified source producers provide every coefficient. This host
// fixture does not stand in for the separate common-publisher GPU qualification.
struct FullLedgerFixture {
  coefficient_test::SolidFixture source;
  tl::fea::ShellBatchBinding shells;
  tl::fea::NodalNodeDomain domain;
  tl::fea::ShellNodeMap mapping;
  tl::fea::type13::Model beams;
  tl::fea::Type13NodeContributions beam_coefficients;
  tl::fea::type25::Model welds;
  tl::fea::SolidNodeContributions solids;
  tl::fea::ElementMassContributions masses;
  tl::fea::NodalCoefficientLedger ledger;
  tl::fea::ShellBatchPlasticityBinding catalog;
  tl::fea::ShellBatchFailureBinding failure;
  tl::fea::NodalRigidAssemblyBinding rigid;
  tl::fea::ShellExecutionBinding execution;
  tl::fea::ShellPhysicalBinding physical;
  std::vector<std::uint64_t> ids,parents;
  std::vector<double> positions,curvature;
  std::vector<s::PrimaryFace> primary;
  std::vector<l::Node> nodes;
  std::vector<l::Main> mains;
  std::vector<l::Secondary> secondary;
  std::vector<l::NormalReference> references;
  std::vector<std::uint32_t> removal_offsets;
  tl::util::HostArena output,scratch;
  s::Snapshot starter;
  explicit FullLedgerFixture(bool with_qbat=false,bool contact_geometry=false)
      : source(contact_geometry),domain(source.Domain()),beams(source.Beams()),
        welds(source.Springs()),
        solids(source.Solids(domain)) {
    auto geometry=source.shell_input.Input();
    auto declared_quads=source.shell_input.q;
    if(contact_geometry) {
      // The GPU coupon explicitly declares centered no-failure layers. Offset
      // glass requires its real Tab1 policy and belongs to the failure gate.
      for(auto& quad:declared_quads)quad.reference.placement=tl::fea::ShellReferencePlacement::Centered;
      geometry.shells.qeph=declared_quads.data();
    }
    const auto shell_report=with_qbat?shells.InitializeFormulations(geometry):shells.Initialize(geometry.shells);
    EXPECT_EQ(shell_report.status,tl::fea::ShellBindingStatus::Success);
    EXPECT_TRUE(mapping.Initialize(shells,domain));
    EXPECT_TRUE(beam_coefficients.Initialize(beams,domain));
    const tl::fea::ElementMassSource point{18000,777,domain.Find(777),.002};
    EXPECT_TRUE(masses.Initialize(domain,{1,1,&point,1}));
    EXPECT_TRUE(ledger.InitializeWithSolids({{&mapping,&welds,&beam_coefficients},&masses,&solids}));
    qbat_catalog_test::Fixture declaration;
    if(!with_qbat) {
      // This distinct synthetic material uses a three-point T3 section with
      // genuine no-failure support. The QBAT control keeps its mandated law.
      declaration.sections[2].through_thickness_points=3;
      declaration.sections[2].formulation=tl::fea::ShellSectionFormulation::LayeredNip3;
    }
    const std::size_t parent_count=with_qbat?4:3;
    const tl::fea::ShellPlasticityParentInput p[]{
      {tl::fea::ShellBindingFamily::Qeph,0,100,1000,1000,1000},
      {tl::fea::ShellBindingFamily::Qeph,1,101,1001,1001,1001},
      {tl::fea::ShellBindingFamily::T3,0,102,2000524,2000524,2000524},
      {tl::fea::ShellBindingFamily::Qbat,0,103,2000524,2000524,2000524}};
    EXPECT_EQ(catalog.InitializeExecutionCatalog(shells,{&declaration.curve,
      declaration.materials.data(),declaration.sections.data(),p,1,3,3,parent_count}).status,
      tl::fea::ShellPlasticityBindingStatus::Success);
    tl::fea::ShellFailureParentInput policies[4];
    for(unsigned i=0;i<4;++i){policies[i].source=p[i];policies[i].policy=tl::fea::ShellFailurePolicy::None;}
    if(with_qbat)for(unsigned i=2;i<4;++i){
      policies[i].policy=tl::fea::ShellFailurePolicy::ConstantAllPoints;
      policies[i].constant.failure_strain=2.5;
    }
    EXPECT_EQ(failure.InitializeExecution(catalog,policies,parent_count).status,tl::fea::ShellPlasticityBindingStatus::Success);
    EXPECT_TRUE(rigid.InitializeEmpty(ledger));
    EXPECT_EQ(execution.Initialize(catalog,ledger,rigid).status,tl::fea::ShellPlasticityBindingStatus::Success);
    EXPECT_TRUE(physical.InitializeExecution({&shells,&catalog,&failure,nullptr},ledger,execution));
    for(std::size_t i=0;i<domain.node_count();++i) {
      const auto& node=domain.nodes()[i];ids.push_back(node.source_id);
      positions.insert(positions.end(),{node.position.x,node.position.y,node.position.z});
      nodes.push_back({node.source_id,7,0});secondary.push_back({std::uint32_t(i),1e6,.001,0});
    }
    const auto add=[&](std::uint64_t id,n::ShellLayout layout,const auto& local) {
      s::PrimaryFace face;face.source_id=id;face.layout=layout;
      for(unsigned k=0;k<4;++k) {
        const auto slot=layout==n::ShellLayout::Triangle3&&k==3?2:k;
        face.nodes[k]=std::uint32_t(domain.Find(shells.active_nodes()[local[slot]].source_id));
      }
      primary.push_back(face);parents.push_back(id);
    };
    if(with_qbat)add(shells.qbat_source_id(0),n::ShellLayout::Quad4,shells.qbat_nodes(0));
    else add(shells.qeph_source_id(0),n::ShellLayout::Quad4,shells.qeph_nodes(0));
    for(std::size_t i=0;i<shells.t3_count();++i)
      add(shells.t3_source_id(i),n::ShellLayout::Triangle3,shells.t3_nodes(i));
    // The second coincident QEPH layer remains a genuine mass contributor,
    // outside this declared synthetic contact selection.
    s::Input input;input.profile=contact_geometry?s::Profile::OrdinaryExteriorMovingMain:s::Profile::OrdinaryExteriorFixedMain;
    input.topology=s::TopologyPolicy::NativeOrdinaryShell;input.node_source_ids=ids.data();input.node_count=ids.size();
    input.positions={positions.data(),std::uint32_t(ids.size()),3,1};
    input.primary=primary.data();input.primary_count=primary.size();input.source_generation=7;
    const auto forecast=s::Preflight(input);
    nodal_empty_test::Fixture::Require(forecast.status==s::Status::Ok&&output.Initialize(forecast.output_bytes)&&
      scratch.Initialize(forecast.scratch_bytes),"Full ledger source topology allocation");
    nodal_empty_test::Fixture::Require(s::BuildStarter(input,{},output,scratch,&starter).status==s::Status::Ok,
      "Full ledger source topology");
    mains.resize(starter.main_count);references.resize(starter.starter.reference_count);
    for(std::size_t i=0;i<mains.size();++i) {
      const auto& original=starter.mains[i];auto& main=mains[i];
      main.global_id=original.global_id;main.segment_type=original.segment_type;main.coefficient=1e6;main.maximum_gap=.001;
      for(unsigned k=0;k<4;++k) {
        main.nodes[k]=original.nodes[k];main.normal_reference[k]=original.normal_reference[k];
        main.neighbors[k]=original.neighbors[k];main.normal_slot[k]=starter.starter.face_normals[4*i+k];main.gap[k]=.001;
      }
    }
    for(std::size_t i=0;i<references.size();++i) {
      const auto& original=starter.starter.references[i];references[i].boundary=original.boundary;
      for(unsigned k=0;k<2;++k)references[i].bisector[k]=original.bisector[k];
    }
    curvature.assign(primary.size(),0);removal_offsets.assign(secondary.size()+1,0);
  }
  n::FixedMainSource Contact() const {
    n::FixedMainSource value;value.source_id=77;value.topology_generation=3;
    value.selection={nodes.data(),nodes.size(),mains.data(),mains.size(),secondary.data(),secondary.size(),
      references.data(),references.size(),{starter.normal_offsets,references.size()+1,starter.normal_mains,starter.normal_incidence_count},
      {removal_offsets.data(),removal_offsets.size(),nullptr,0},7};
    value.primary_main_count=primary.size();value.primary_parent_ids=parents.data();value.primary_curvature=curvature.data();
    value.margin=.01;value.force_packet_size=128;value.native_workers=1;return value;
  }
  static n::TransactionConfig Config() {
    auto c=Fixture::Config();c.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
    c.physical_source=n::PhysicalSourceProfile::CompleteBoundLedger;return c;
  }
};
} // namespace type25_source_test
