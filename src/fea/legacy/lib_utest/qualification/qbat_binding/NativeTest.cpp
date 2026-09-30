#include "Fixture.h"
#include "OriginalFixture.h"
#include "../qbat/NativeOracle.h"
#include "../native/t3/T3Reference.h"
#include <algorithm>

namespace qbat_binding_test {
namespace native_t3=tl::qualification::t3;
void Near(double actual,double native) {
  ASSERT_TRUE(std::isfinite(native));
  EXPECT_NEAR(actual,native,3e-12*std::max(std::abs(actual),std::abs(native))+1e-24);
}
void Agreement(const Mass& actual,const Mass& native) {
  Near(actual.mass,native.mass);
  Near(actual.isotropic_inertia,native.isotropic_inertia);
  Near(actual.physical_inertia,native.physical_inertia);
  Near(actual.added_inertia,native.added_inertia);
}
TEST(QbatBindingNative, CompleteOriginalCollectionUsesIndependentNativeQbatAndT3Coefficients) {
  const OriginalCollection input;
  Binding binding;
  ASSERT_EQ(binding.InitializeFormulations(input.Input(),fe::ShellHostBindingLimits::Vehicle()).status,Status::Success);
  std::vector<Mass> nodes(binding.node_count());
  Mass total,qbat_total;
  native_t3::ReferenceInput triangle;
  for(unsigned n=0;n<3;++n) {
    triangle.position[n]=input.triangle.reference.position[n];
    triangle.node_ids[n]=input.triangle.reference.node_ids[n];
  }
  triangle.density=input.triangle.reference.density;
  triangle.thickness=input.triangle.reference.thickness;
  triangle.young_modulus=input.triangle.reference.young_modulus;
  triangle.poisson_ratio=input.triangle.reference.poisson_ratio;
  native_t3::Reference native_triangle;
  ASSERT_EQ(native_t3::Initialize(triangle,native_triangle),native_t3::Status::kSuccess);
  for(unsigned n=0;n<3;++n) {
    const auto term=Term(native_triangle.data(),n);
    Add(nodes[input.triangle.nodes[n]],term);
    Add(total,term);
    Agreement(Term(binding.t3_reference(0),n),term);
  }
  for(std::size_t i=0;i<input.quads.size();++i) {
    SCOPED_TRACE(input.quads[i].source_parent_id);
    const auto native=qbat_test::NativeReference(input.quads[i].reference);
    const auto actual=qbat_test::Values(binding.qbat_reference(i));
    for(unsigned k=0;k<actual.size();++k) Near(actual[k],native[k]);
    if(::testing::Test::HasFailure()) return;
    // Native reference packet: mass, physical J, added J, TOTAL J at 30:33.
    const Mass term{native[30],native[33],native[31],native[32]};
    for(unsigned n=0;n<4;++n) {
      Agreement(Term(binding.qbat_reference(i).quadrilateral(),n),term);
      Add(nodes[input.quads[i].nodes[n]],term);
      Add(total,term);
      Add(qbat_total,term);
    }
  }
  for(std::size_t n=0;n<nodes.size();++n) Agreement(binding.nodes()[n].native,nodes[n]);
  Agreement(binding.totals(),total);
  Agreement(binding.qbat_totals(),qbat_total);
}
} // namespace qbat_binding_test
