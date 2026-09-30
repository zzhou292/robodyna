#include "../ReferenceStorage.h"
#include "../QbatReferenceInput.h"
#include "ReferenceComparison.h"
#include "modelio/vehicle_sections/tests/MidlayerFixture.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include "lib_src/elements/qbat/QbatReference.h"

namespace crash::cases::vehicle_startup::test {
namespace {
auto Material() {
    return modelio::vehicle::resolution::ReadMidlayer(modelio::vehicle::test::MidlayerFields(),{1000,.001,1});
}
auto Packet() {
    const auto declaration=Material();
    const modelio::assembly::SourceReferenceNode nodes[4]{{7,{-0.,0,0}},{3,{.01,0,0}},
        {11,{.01,.008,.00001}},{13,{0,.008,0}}};
    const auto quad=modelio::assembly::PackShellReference<tl::fea::qeph::ReferenceInput>(
        nodes,declaration.material,declaration.section);
    const auto native=modelio::assembly::detail::NativeMaterial(declaration.material,
        modelio::assembly::detail::NativeLaw44Rate::FilteredZeroC);
    return detail::OriginalMidlayerQbatInput(quad,native);
}
}
TEST(VehicleMidlayerReference, QualifiedQbatOptionsAndVirginMaterialPreserveSourceCoordinates) {
    const auto input=Packet();
    Same(input.quadrilateral.position[0].x,-0.0);
    EXPECT_EQ(input.options.ihbe,11);
    EXPECT_EQ(input.options.nptr,2);
    EXPECT_EQ(input.options.npts,2);
    EXPECT_EQ(input.options.nptt,1);
    EXPECT_EQ(input.options.layers,1);
    EXPECT_EQ(input.options.membrane_viscosity,0);
    EXPECT_EQ(input.options.numerical_viscosity,0);
    const double young=input.quadrilateral.young_modulus,nu=input.quadrilateral.poisson_ratio;
    Same(input.initial_a11_pa,young/(1-nu*nu));
    tl::fea::qbat::Reference expected;
    ASSERT_EQ(tl::fea::qbat::InitializeReference(input,expected),tl::fea::qbat::Status::kSuccess);
    detail::ReferenceStorage result;
    detail::Append(result,{},input);
    ASSERT_EQ(result.qbat.size(),1);
    EXPECT_EQ(result.rows[0].family,ReferenceFamily::Qbat);
    Same(result.qbat[0].quadrilateral(),expected.quadrilateral());
    Same(result.qbat[0].coefficients().unscaled_element_dt_s,expected.coefficients().unscaled_element_dt_s);
    RecordProperty("sizeof_qbat_reference",std::to_string(sizeof(tl::fea::qbat::Reference)));
}
TEST(VehicleMidlayerReference, InterleavedStorageAndRejectedPacketDoNotUseTopologyIndices) {
    const auto input=Packet();
    detail::ReferenceStorage result;
    ReferenceRow row;
    row.element_id=90;
    detail::Append(result,row,input.quadrilateral);
    row.element_id=80;
    detail::Append(result,row,input);
    auto bad=input;
    bad.quadrilateral.position[3]=bad.quadrilateral.position[2];
    row.element_id=70;
    detail::Append(result,row,bad);
    row.element_id=60;
    detail::AppendUnresolved(result,row);
    row.element_id=50;
    detail::Append(result,row,input.quadrilateral);
    row.element_id=40;
    detail::Append(result,row,input);
    ASSERT_EQ(result.rows.size(),6);
    EXPECT_EQ(result.rows[0].reference_index,0);
    EXPECT_EQ(result.rows[1].reference_index,0);
    EXPECT_EQ(result.rows[2].reference_index,SIZE_MAX);
    EXPECT_EQ(result.rows[3].reference_index,SIZE_MAX);
    EXPECT_EQ(result.rows[4].reference_index,1);
    EXPECT_EQ(result.rows[5].reference_index,1);
    EXPECT_EQ(result.first_error,2);
    EXPECT_EQ(result.counts.qbat_attempted,3);
    EXPECT_EQ(result.counts.qbat_succeeded,2);
    EXPECT_EQ(result.counts.rejected,1);
    Same(result.qbat[0].quadrilateral(),result.qbat[1].quadrilateral());
}
TEST(VehicleMidlayerReference, MaterialAndPlacementMismatchLeavesPriorInputUnchanged) {
    const auto input=Packet();
    auto visible=input;
    const auto declaration=Material();
    auto native=modelio::assembly::detail::NativeMaterial(declaration.material,
        modelio::assembly::detail::NativeLaw44Rate::FilteredZeroC);
    native.poisson_ratio=.3;
    EXPECT_THROW(visible=detail::OriginalMidlayerQbatInput(input.quadrilateral,native),std::runtime_error);
    SameInput(visible.quadrilateral,input.quadrilateral);
    native.poisson_ratio=declaration.material.poisson_ratio;
    auto placed=input.quadrilateral;
    placed.placement=tl::fea::ShellReferencePlacement::TopReferencePlane;
    EXPECT_THROW(visible=detail::OriginalMidlayerQbatInput(placed,native),std::runtime_error);
    visible=detail::OriginalMidlayerQbatInput(input.quadrilateral,native);
    Same(visible.initial_a11_pa,input.initial_a11_pa);
}
} // namespace crash::cases::vehicle_startup::test
