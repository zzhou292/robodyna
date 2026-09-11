// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "../qbat_binding/OriginalFixture.h"
#include "../qbat/source_fixture/YarisQbatSourceFixture.h"

namespace qbat_catalog_test {
namespace original=yaris_qbat_source_fixture;
struct SourceCatalog {
  qbat_binding_test::OriginalCollection geometry;
  fe::ShellPlasticityMaterialInput material=Midlayer();
  fe::ShellPlasticitySectionInput section=MidlayerSection();
  std::vector<fe::ShellPlasticityParentInput> parents;
  std::vector<fe::ShellFailureParentInput> failures;
  SourceCatalog() {
    const fe::ShellPlasticityParentInput triangle{Family::T3,0,2357656,2000524,2000524,2000524};
    bool inserted=false;
    for(std::size_t i=0;i<geometry.QuadCount;++i) {
      // Original triangle line is authenticated by the shared fixture manifest.
      if(!inserted&&original::quads[i].source_line>274492) {
        parents.push_back(triangle);
        inserted=true;
      }
      parents.push_back({Family::Qbat,i,original::quads[i].id,2000524,2000524,2000524});
    }
    if(!inserted) parents.push_back(triangle);
    for(const auto& p:parents) failures.push_back(Failure(p));
  }
  fe::ShellBatchPlasticityBindingInput Input() const {
    return {nullptr,&material,&section,parents.data(),0,1,1,parents.size()};
  }
};

} // namespace qbat_catalog_test
