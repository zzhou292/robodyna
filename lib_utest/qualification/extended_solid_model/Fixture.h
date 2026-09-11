// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/solid_model/TestSupport.h"
#include "lib_src/elements/solid18/law44/Reference.h"
#include "lib_src/elements/solid18/total_strain/Reference.h"
#include "lib_src/materials/law44/solid/Prepare.h"
#include "lib_src/materials/law90/Prepare.h"
#include <gtest/gtest.h>
namespace extended_model_test {
namespace fe = tl::fea;
namespace s = fe::solids;
namespace rear = fe::solid18::law44;
namespace foam = fe::solid18::total_strain;
namespace law44 = tl::material::law44::solid;
namespace law90 = tl::material::law90;
using solid_model_test::Require;
using solid_model_test::Bits;
struct Fixture : solid_model_test::Fixture {
  double rear_x[3]{0,.2,.4}, rear_y[3]{270e6,350e6,450e6};
  double foam_x[3]{0,.2,.4}, foam_y[3]{0,10e6,50e6};
  law90::PreparationInput foam_input;
  std::vector<s::Input18Law44> input44;
  std::vector<s::Input18Law90> input90;
  Fixture() {
    auto a = input18[0].reference.input();
    a.source_element_id = 104; a.source_part_id = 16;
    a.source_section_id = 17; a.source_material_id = 44;
    a.profile = rear::Profile();
    a.density_kg_m3 = 7.89e-9*1e12;
    for (auto& nid : a.source_node_id) nid += 1000;
    a.source_node_id[5] = a.source_node_id[4];
    a.source_node_id[7] = a.source_node_id[6];
    a.position_m[4] = a.position_m[5] = {.01,0,.02};
    a.position_m[6] = a.position_m[7] = {.01,.02,.02};
    for (unsigned n = 0; n < 8; ++n) {
      bool seen = false;
      for (unsigned k = 0; k < n; ++k) seen |= a.source_node_id[k] == a.source_node_id[n];
      if (!seen) nodes.push_back({a.source_node_id[n],a.position_m[n]});
    }
    input44.resize(1);
    Require(rear::InitializeReference(a,input44[0].reference) == fe::solid18::Status::Success);
    const law44::Material material{210e9,.3,a.density_kg_m3,8000,8,10000,
      law44::WorkingUnits::TonneMillimetreSecond};
    Require(law44::Prepare(material,{rear_x,rear_y,3},input44[0].material) == law44::Status::Ok);
    auto b = input18[0].reference.input();
    b.source_element_id = 105; b.source_part_id = 18;
    b.source_section_id = 19; b.source_material_id = 90;
    b.profile = foam::Law90Profile(); b.density_kg_m3 = 772;
    input90.resize(1);
    Require(foam::InitializeReference90(b,input90[0].reference) == fe::solid18::Status::Success);
    foam_input.density_kg_m3 = 772;
    foam_input.card_young_pa = 72e6;
    foam_input.contact_modulus_pa = 4e6;
    PrepareFoam();
  }
  void PrepareFoam() {
    Require(law90::PrepareSI(foam_input,{foam_x,foam_y,3},input90[0].material) == law90::Status::Ok);
  }
  s::ModelInput Input() const {
    auto value = solid_model_test::Fixture::Input();
    value.solid18_law44 = {input44.empty()?nullptr:input44.data(),input44.size()};
    value.solid18_law90 = {input90.empty()?nullptr:input90.data(),input90.size()};
    value.profile = s::ModelProfile::ExtendedLaw44Law90;
    return value;
  }
  void Repeat44() {
    auto next = input44[0];
    auto source = next.reference.input(); source.source_element_id += 100;
    Require(rear::InitializeReference(source,next.reference) == fe::solid18::Status::Success);
    input44.push_back(next);
  }
  void Repeat90() {
    auto next = input90[0];
    auto source = next.reference.input(); source.source_element_id += 100;
    Require(foam::InitializeReference90(source,next.reference) == fe::solid18::Status::Success);
    input90.push_back(next);
  }
};
inline void Empty(const s::Model& model) {
  EXPECT_FALSE(model.prepared());
  EXPECT_EQ(model.domain(),nullptr);
  EXPECT_EQ(model.contributions(),nullptr);
  EXPECT_EQ(model.solid18_law44().size(),0u);
  EXPECT_EQ(model.solid18_law90().size(),0u);
  EXPECT_EQ(model.owned_payload_bytes(),sizeof(model));
}
} // namespace extended_model_test
