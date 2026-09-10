#include "SourceAssemblyBindingTestSupport.h"

namespace crash::cases::source_assembly::test {
TEST(SourceBracketBindings, UndeclaredInternalWeldCannotBecomeReleasedFreeFlight) {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_BRACKET_INVENTORY");
    ASSERT_NE(path,nullptr);
    const auto source_model=source::SourceAssembly::Read(path,source::PinnedYarisSevenPartInventory());
    ASSERT_EQ(source_model.data().internal_spotwelds.size(),1u);
    for (unsigned retry=0;retry<2;++retry) {
        try {
            (void)SourceAssemblyBindings::Prepare(source_model,Options());
            FAIL()<<"An internal source weld must require its qualified contribution";
        } catch(const SourceAssemblyBindingError& error) {
            EXPECT_EQ(error.stage,SourceAssemblyBindingStage::Input);
            EXPECT_NE(std::string(error.what()).find("connector contribution"),std::string::npos);
        }
        EXPECT_EQ(source_model.data().internal_spotwelds.front().record.id,2101297u);
        EXPECT_EQ(source_model.data().parents.size(),959u);
    }
}
} // namespace crash::cases::source_assembly::test
