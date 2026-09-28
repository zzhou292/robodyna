#include "../Controls.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::activity {
namespace s=tlfea::contact::radioss_type25::startup;
TEST(NativeActivityDeclaration, OriginalSelfAndDeclaredWallKeepDifferentDeletionPolicies) {
    values::RawControls self,wall;self.reader_idel=1;
    const auto a=values::Self(self,s::SolidErosion::Enabled),b=values::Wall(wall,0);
    EXPECT_EQ(a.deletion,native::Deletion::ContainingElement);EXPECT_FALSE(a.keep_disconnected_nodes);
    EXPECT_EQ(a.solid_erosion,s::SolidErosion::Enabled);
    EXPECT_EQ(b.deletion,native::Deletion::Disabled);EXPECT_FALSE(b.keep_disconnected_nodes);
    EXPECT_EQ(b.solid_erosion,s::SolidErosion::Disabled);
}
TEST(NativeActivityDeclaration, UnknownOrDifferentSourceControlsRejectRatherThanDefault) {
    values::RawControls raw;
    for(int value:{-1,0,2,3}){raw.reader_idel=value;EXPECT_THROW(values::Self(raw,s::SolidErosion::Enabled),std::exception);}
    raw.reader_idel=1;EXPECT_THROW(values::Self(raw,s::SolidErosion::Disabled),std::exception);
    EXPECT_THROW(values::Wall(raw,0),std::exception);
    raw.reader_idel=0;
    EXPECT_THROW(values::Wall(raw,1),std::exception);
}
TEST(NativeActivityDeclaration, DeclarationDoesNotMutateOriginalControlProvenance) {
    values::RawControls raw;raw.reader_idel=1;raw.stfac=.75;raw.slsfac=.125;raw.source_death_blank=true;
    (void)values::Self(raw,s::SolidErosion::Enabled);
    EXPECT_EQ(raw.reader_idel,1);EXPECT_EQ(raw.stfac,.75);EXPECT_EQ(raw.slsfac,.125);EXPECT_TRUE(raw.source_death_blank);
}
}
