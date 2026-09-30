#include "OriginalFixture.h"
#include "../qbat/source_fixture/YarisQbatSourceFixture.h"
#include <iterator>
#include <stdexcept>

namespace qbat_binding_test {
namespace original=yaris_qbat_source_fixture;
OriginalCollection::OriginalCollection() {
  static_assert(std::size(original::nodes)==NodeCount);
  static_assert(std::size(original::quads)==QuadCount);
  static_assert(original::part_id==2000524);
  quads.resize(QuadCount);
  for(std::size_t i=0;i<QuadCount;++i) {
    const auto& row=original::quads[i];
    auto& parent=quads[i];
    parent.source_parent_id=row.id;
    auto& q=parent.reference.quadrilateral;
    for(unsigned n=0;n<4;++n) {
      const auto& source=original::nodes[row.node[n]];
      parent.nodes[n]=row.node[n];
      q.node_ids[n]=source.id;
      q.position[n]={source.position_m[0],source.position_m[1],source.position_m[2]};
    }
    // Existing authenticated source manifest: virgin native geometric inputs
    // only. No source material/failure/connected-runtime admission is implied.
    q.density=1000;
    q.young_modulus=250e6;
    q.poisson_ratio=.35;
    q.thickness=original::thickness_m;
    parent.reference.initial_a11_pa=q.young_modulus/(1-q.poisson_ratio*q.poisson_ratio);
  }
  triangle.source_parent_id=2357656;
  triangle.reference.density=1000;
  triangle.reference.young_modulus=250e6;
  triangle.reference.poisson_ratio=.35;
  triangle.reference.thickness=original::thickness_m;
  // Exact separately retained original T3 record in the same manifest.
  const std::uint64_t ids[]{2300357,2300138,2300139};
  for(unsigned n=0;n<3;++n) {
    std::size_t i=0;
    for(;i<NodeCount;++i) if(original::nodes[i].id==ids[n]) break;
    if(i==NodeCount) throw std::invalid_argument("Original triangle source node is absent");
    const auto& source=original::nodes[i];
    triangle.nodes[n]=i;
    triangle.reference.node_ids[n]=source.id;
    triangle.reference.position[n]={source.position_m[0],source.position_m[1],source.position_m[2]};
  }
}
tl::fea::ShellFormulationCollectionInput OriginalCollection::Input() const {
  return {{nullptr,&triangle,0,1,NodeCount},quads.data(),quads.size()};
}
} // namespace qbat_binding_test
