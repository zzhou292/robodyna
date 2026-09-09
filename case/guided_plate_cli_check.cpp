#include "chrono/core/ChMatrix.h"
#include "GuidedPlateCli.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <vector>

namespace {
using namespace crash::case_data;
using Backend=tlfea::contact::Q4PlanarIntegrationBackend;
namespace fs=std::filesystem;
GuidedPlateCommand Parse(std::initializer_list<const char*> args) {
    std::vector<const char*> values(args);return ParseGuidedPlateCommand(static_cast<int>(values.size()),values.data());
}
struct Temp {
    fs::path path;
    Temp() {
        const auto pattern=(fs::temp_directory_path()/"guided-cli-XXXXXX").string();
        std::vector<char> data(pattern.begin(),pattern.end());data.push_back(0);
        const auto* made=::mkdtemp(data.data());if(!made)throw std::runtime_error("Cannot create CLI test directory");path=made;
    }
    ~Temp(){std::error_code error;fs::remove_all(path,error);}
};
TEST(GuidedPlateCli, LegacyFormsRemainScalarAndTyped) {
    auto command=Parse({"guided","run","wall","study","2"});
    EXPECT_EQ(command.kind,GuidedPlateCommandKind::Run);EXPECT_EQ(command.run.config.refinement,2u);
    EXPECT_EQ(command.run.config.integration_backend,Backend::ScalarDyadicSquares);
    EXPECT_EQ(command.run.wall_kind,WallTessellationKind::Original);EXPECT_FALSE(command.run.bundle);EXPECT_FALSE(command.run.wall_provenance);
    command=Parse({"guided","run","wall","study","1","bundle"});EXPECT_EQ(*command.run.bundle,"bundle");
    command=Parse({"guided","compare","coarse","fine","report"});
    EXPECT_EQ(command.kind,GuidedPlateCommandKind::CompareRefinement);EXPECT_EQ(command.coarse_study,"coarse");
    EXPECT_EQ(command.fine_study,"fine");EXPECT_EQ(command.refinement_report,"report");
}
TEST(GuidedPlateCli, ExplicitTransformAndBackendDoNotCreateAutomaticRuns) {
    auto command=Parse({"guided","run","wall","study","4","--wall=subdivide","--wall-provenance=proof","--contact-integration=rectangular"});
    EXPECT_EQ(command.run.wall_kind,WallTessellationKind::UniformFour);EXPECT_EQ(command.run.config.refinement,4u);
    EXPECT_EQ(command.run.config.integration_backend,Backend::RectangularDyadic);EXPECT_EQ(*command.run.wall_provenance,"proof");
    EXPECT_FALSE(command.run.bundle);
    command=Parse({"guided","run","wall","study","1","bundle","--wall=original","--wall-provenance=proof","--contact-integration=scalar"});
    EXPECT_EQ(*command.run.bundle,"bundle");EXPECT_EQ(command.run.config.integration_backend,Backend::ScalarDyadicSquares);
    command=Parse({"guided","run","wall","study","1","--wall-provenance=proof","--wall=flip"});
    EXPECT_EQ(command.run.wall_kind,WallTessellationKind::FlipConvexPairs);
    command=Parse({"guided","compare-wall","wall","derived","derived-proof","canonical","canonical-proof","report"});
    EXPECT_EQ(command.kind,GuidedPlateCommandKind::CompareWall);const auto& p=command.wall_comparison;
    EXPECT_EQ(p.wall,"wall");EXPECT_EQ(p.derived_study,"derived");EXPECT_EQ(p.derived_provenance,"derived-proof");
    EXPECT_EQ(p.canonical_study,"canonical");EXPECT_EQ(p.canonical_provenance,"canonical-proof");EXPECT_EQ(p.report,"report");
}
TEST(GuidedPlateCli, MissingAmbiguousOrUnknownOptionsRejectBeforeFilesystemOrDevice) {
    for(const auto* bad:{"0","3","01","-1","all"})EXPECT_THROW(Parse({"g","run","w","s",bad}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","--wall=flip"}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","bundle","--wall=flip","--wall-provenance=p"}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","--wall=original","--wall=original"}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","--wall=unknown"}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","--wall-provenance="}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","--wall-provenance=p","--wall-provenance=q"}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","--contact-integration=scalar","--contact-integration=rectangular"}),std::runtime_error);
    for(const auto* bad:{"--contact-integration=","--contact-integration=auto","--help","--wall"})
        EXPECT_THROW(Parse({"g","run","w","s","1",bad}),std::runtime_error);
    EXPECT_THROW(Parse({"g","run","w","s","1","bundle","another"}),std::runtime_error);
    EXPECT_THROW(Parse({"g","compare-wall","w","d","dp","c","cp"}),std::runtime_error);
    EXPECT_THROW(ParseGuidedPlateCommand(2,nullptr),std::runtime_error);
    const char* null_arg[]{"g",nullptr};EXPECT_THROW(ParseGuidedPlateCommand(2,null_arg),std::runtime_error);
}
TEST(GuidedPlateCli, PreflightRejectsExistingAliasedAndDanglingOutputsWithoutInitializingCase) {
    Temp root;GuidedPlateRunOptions options;options.wall=root.path/"unread-wall";options.study=root.path/"study";
    EXPECT_NO_THROW(CheckGuidedPlateRunOptions(options));
    auto bad=options;bad.wall_kind=WallTessellationKind::FlipConvexPairs;EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    bad.wall_provenance=root.path/"proof";EXPECT_NO_THROW(CheckGuidedPlateRunOptions(bad));
    bad.bundle=root.path/"bundle";EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    bad=options;bad.config.integration_backend=static_cast<Backend>(99);EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    bad=options;bad.wall_provenance=root.path/"."/"study";EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    fs::create_directory(root.path/"real");fs::create_directory_symlink(root.path/"real",root.path/"alias");
    bad=options;bad.study=root.path/"real"/"study";bad.wall_provenance=root.path/"alias"/"study";
    EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    bad=options;bad.bundle=bad.study;EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    bad=options;bad.study=root.path/"missing"/"study";EXPECT_THROW(CheckGuidedPlateRunOptions(bad),std::runtime_error);
    fs::create_symlink(root.path/"absent-target",options.study);EXPECT_THROW(CheckGuidedPlateRunOptions(options),std::runtime_error);
    fs::remove(options.study);crash::output::WriteBytes(options.study,"preserved");
    GuidedPlateCommand command;command.run=options;EXPECT_THROW(ExecuteGuidedPlateCommand(command),std::runtime_error);
    EXPECT_EQ(crash::output::ReadBounded(options.study,100),"preserved");
    EXPECT_FALSE(fs::exists(options.wall));EXPECT_FALSE(fs::exists(root.path/"bundle"));
}
} // namespace
