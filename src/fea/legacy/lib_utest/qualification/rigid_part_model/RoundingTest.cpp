// SPDX-License-Identifier: AGPL-3.0-or-later
#include "HighPrecision.h"

namespace rigid_part_model_test {
TEST(RigidPartModel, Binary128AbsoluteTermsBoundOriginalAndMergedRawValues) {
  for(const double shift:{0.,1e5}) for(const double primary_scale:{1000.,1e20}) {
    Fixture f;
    f.units.mass_to_kg=primary_scale;
    // Preserve exact shell coordinate association; shift the point-only bodies.
    for(std::size_t n=5;n<f.nodes.size();++n) {
      f.nodes[n].position.x+=shift;f.nodes[n].position.y-=shift;
    }
    const auto ledger=f.Ledger();const auto topology=f.Topology();
    Model model;const auto report=model.Initialize(*topology,ledger,f.units);
    ASSERT_TRUE(report)<<report.message;
    std::vector<precise::Body> original;
    for(std::size_t p=0;p<f.part.size();++p) {
      original.push_back(precise::Original(Packet(f,ledger,p)));
      precise::Check(model.original_bodies()[p].raw,original.back());
    }
    for(std::size_t i=0;i<model.roots().size();++i) {
      const auto& root=topology->roots()[i];
      const auto expected=root.child_part_index==SIZE_MAX?original[root.part_index]:
        precise::Merge(original[root.part_index],original[root.child_part_index]);
      precise::Check(model.roots()[i].value.raw,expected);
      const auto& final=model.roots()[i].value;
      for(unsigned k=0;k<9;++k)
        EXPECT_NEAR(final.effective_tensor.v[k],final.raw.tensor.v[k]+final.regularization.tensor_added.v[k],
          256*std::numeric_limits<double>::epsilon()*std::max(1.,std::abs(final.raw.tensor.v[k])));
    }
  }
}
} // namespace rigid_part_model_test
