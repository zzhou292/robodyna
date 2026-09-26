// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/materials/law36/Prepare.h"
#include "lib_src/materials/law42/Prepare.h"

namespace physical_publication_test {
Fixture::Fixture(bool surface,double failure,bool contact_geometry,
                 ContactConstraintLayout constraints,
                 bool interior_edge_contact, double adjacent_apex_x,
                 fe::ShellBatchStartup declared_startup, bool separate_adjacent_contact,
                 bool detached_adjacent_triangle)
    : source(contact_geometry,
             constraints == ContactConstraintLayout::SameMergedParts ||
             constraints == ContactConstraintLayout::MergedPartAndPlain,
             interior_edge_contact, adjacent_apex_x, separate_adjacent_contact, detached_adjacent_triangle),
      startup(declared_startup), surface_rigid(surface),t3_failure(failure),
      contact_constraints(constraints) {
  source.nodes.push_back({778,{.06,-.01,.003}});
  source.nodes.push_back({901,{.02,.01,.001}});
  EXPECT_TRUE(domain.Initialize({1,source.nodes.data(),source.nodes.size()}));
  EXPECT_TRUE(shells.Initialize(source.shells,domain));
  PrepareSources();
  // Only the two actual QEPH parents witness 10--13 when contact QBAT has
  // its own 20--23 source nodes. Keep the real CIN attachment and masters.
  if (separate_adjacent_contact) {
    ranges[0].count = 2;
    witness_count = 2;
  }
  PrepareConstraints();
  PrepareMaterials();
  const auto count = domain.node_count();
  x.resize(3*count); v.resize(3*count); w.resize(3*count); q.resize(4*count);
  m.resize(count); j.resize(count); im.resize(count); ij.resize(count);
  fixed.resize(count); rotation_fixed.resize(count); present.resize(count);
  for (std::size_t node = 0; node < count; ++node) {
    const auto p = domain.nodes()[node].position;
    x[3*node] = p.x; x[3*node+1] = p.y; x[3*node+2] = p.z;
    v[3*node] = startup.uniform_velocity.x;
    v[3*node+1] = startup.uniform_velocity.y;
    v[3*node+2] = startup.uniform_velocity.z;
    q[4*node] = 1;
    m[node] = ledger.nodes()[node].coefficients.mass;
    j[node] = ledger.nodes()[node].coefficients.isotropic_inertia;
    present[node] = j[node] > 0 || rigid.FindMember(node);
    im[node] = m[node] > 0 ? 1/m[node] : 0;
    ij[node] = j[node] > 0 ? 1/j[node] : 0;
  }
  const auto secondary = contact_constraints ==
          ContactConstraintLayout::SurfaceCinSecondary
      ? domain.Find(14) : domain.Find(901);
  im[secondary] = 0;
  ij[secondary] = 0;
}
void Fixture::PrepareSources() {
  auto property = source.spring_property;
  for (auto& damping : property.property.damping) damping = 0;
  const bool rigid_contact =
      contact_constraints == ContactConstraintLayout::SameMergedParts ||
      contact_constraints == ContactConstraintLayout::MergedPartAndPlain;
  fe::type25::ConnectionInput connections[2];
  for (unsigned row = 0; row < 2; ++row) {
    auto& connection = connections[row];
    connection.source_element_id = 19000+row;
    const std::uint64_t ends[]{
        901,row && !surface_rigid && !rigid_contact ? 14u : 12u};
    for (unsigned slot = 0; slot < 2; ++slot) {
      const auto node = domain.Find(ends[slot]);
      connection.global_node[slot] = node;
      connection.source_node_id[slot] = ends[slot];
      connection.position[slot] = domain.nodes()[node].position;
    }
  }
  fe::type25::ModelInput weld_input;
  weld_input.source_instance_id = 1;
  weld_input.global_node_count = domain.node_count();
  weld_input.source_units = {1,1,1};
  weld_input.properties = &property;
  weld_input.property_count = 1;
  weld_input.connections = connections;
  weld_input.connection_count = 2;
  EXPECT_TRUE(welds.Initialize(weld_input));

  type13_test::Fixture beam_material;
  auto native = beam_material.Input();
  native.units = {1,1,1};
  const fe::type13::ModelPropertyInput declaration{200,native};
  fe::type13::ModelReport beam_report;
  if (rigid_contact) {
    fe::type13::ModelNode beam_nodes[4];
    const std::uint64_t ids[]{9302,9303,9304,9305};
    for (unsigned slot = 0; slot < 4; ++slot) {
      const auto node = domain.Find(ids[slot]);
      beam_nodes[slot] = {ids[slot],node,domain.nodes()[node].position};
    }
    // The pair collectively gives every non-contact CIN master a genuine
    // rotational source without assigning any contact node to rigid+CIN.
    const fe::type13::ModelConnection connections[]{
        {8000,0,{0,1,2}},{8001,0,{2,3,0}}};
    beam_report = beams.Initialize(
        {1,{1,1,1},beam_nodes,&declaration,connections,
         4,1,2,domain.node_count()});
  } else {
    fe::type13::ModelNode beam_nodes[3];
    const std::uint64_t ids[]{901,13,12};
    for (unsigned slot = 0; slot < 3; ++slot) {
      const auto node = domain.Find(ids[slot]);
      beam_nodes[slot] = {ids[slot],node,domain.nodes()[node].position};
    }
    const fe::type13::ModelConnection connection{8000,0,{0,1,2}};
    beam_report = beams.Initialize(
        {1,{1,1,1},beam_nodes,&declaration,&connection,
         3,1,1,domain.node_count()});
  }
  EXPECT_TRUE(beam_report) << beam_report.message;
  EXPECT_TRUE(beam_coefficients.Initialize(beams,domain));

  fe::solids::Input18 input18;
  fe::solids::Input24 input24;
  fe::solids::Input6z input6z;
  EXPECT_EQ(fe::solid18::InitializeReference(source.a,input18.reference),fe::solid18::Status::Success);
  EXPECT_EQ(fe::solid24::InitializeReference(source.b,input24.reference),fe::solid24::Status::Success);
  EXPECT_EQ(fe::solid6z::InitializeReference(source.c,input6z.reference),fe::solid6z::Status::Success);
  const double strain[]{0,.2,.4},stress[]{1e6,2e6,3e6};
  EXPECT_EQ(tl::material::law36::Prepare(100e6,.3,source.a.density_kg_m3,{strain,stress,3},input18.material),
      tl::material::law36::Status::Ok);
  EXPECT_EQ(tl::material::law42::Prepare(24e6,.463,source.b.density_kg_m3,1e26,input24.material),
      tl::material::law42::Status::Ok);
  input6z.material = input24.material;
  EXPECT_TRUE(solids.Initialize(domain,{1,{&input18,1},{&input24,1},{&input6z,1}}));
  const fe::ElementMassSource masses[]{{18000,777,domain.Find(777),0},
      {18001,778,domain.Find(778),.002},{18002,55,domain.Find(55),.002}};
  EXPECT_TRUE(point.Initialize(domain,{1,1000,masses,3}));
  EXPECT_TRUE(ledger.InitializeWithSolids({{&shells,&welds,&beam_coefficients},&point,solids.contributions()}));
}
} // namespace physical_publication_test
