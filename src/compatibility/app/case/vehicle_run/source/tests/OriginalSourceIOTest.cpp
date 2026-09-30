#include "../OriginalSourceIO.h"
#include "FrozenReadOriginal.h"
#include "output/full_shell/static_bundle/Types.h"
#include "output/full_shell/tests/TestSupport.h"
#include <utility>
namespace crash::cases::vehicle_run::detail::test {
namespace files = output::full_shell::test;
namespace {
struct Outcome { bool accepted=false; std::string bytes, error; };
template<class Read> Outcome Inspect(Read read) {
    try { return {true,read(),{}}; }
    catch(const std::exception& error) { return {false,{},error.what()}; }
}
Outcome CompareRead(const std::filesystem::path& path,std::size_t count,const std::string& hash) {
    const auto original=Inspect([&] { return frozen::ReadOriginal(path,count,hash.c_str()); });
    const auto current=Inspect([&] { return ReadOriginal(path,count,hash.c_str()); });
    EXPECT_EQ(current.accepted,original.accepted);
    EXPECT_EQ(current.bytes,original.bytes);
    EXPECT_EQ(current.error,original.error);
    return current;
}
}
TEST(OriginalSourceIO, ExactEmptyBinaryAndChunkBoundaryBytesMatchFrozenReader) {
    files::Directory directory;
    const auto path=directory.path/"member.key";
    for(const auto& bytes : {std::string{},std::string("a\0b\r\n",5),std::string(4096,'x')}) {
        files::Overwrite(path,bytes);
        const auto result=CompareRead(path,bytes.size(),output::Sha256(bytes));
        ASSERT_TRUE(result.accepted) << result.error;
        EXPECT_EQ(result.bytes,bytes);
    }
}
TEST(OriginalSourceIO, ShortAndSameSizeWrongHashPreserveIdentityRejection) {
    files::Directory directory;
    const auto path=directory.path/"member.key";
    files::Overwrite(path,"abc");
    for(const auto& request : {std::make_pair(std::size_t{4},output::Sha256("abc")),
        std::make_pair(std::size_t{3},output::Sha256("abd")),std::make_pair(std::size_t{3},std::string{})}) {
        const auto result=CompareRead(path,request.first,request.second);
        EXPECT_FALSE(result.accepted);
        EXPECT_EQ(result.error,"Pinned original source member or declaration identity differs");
    }
}
TEST(OriginalSourceIO, MissingAndOversizedFilesRetainUnderlyingFailurePriority) {
    files::Directory directory;
    const auto path=directory.path/"member.key";
    auto result=CompareRead(path,4,output::Sha256("abcd"));
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error,"Required artifact/input could not be opened");
    files::Overwrite(path,"abcde");
    result=CompareRead(path,4,output::Sha256("abcde"));
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error,"Artifact/input exceeds byte cap");
}
TEST(OriginalSourceIO, CanonicalRejectsBorrowedMemberBeforeReadingAnySourceFiles) {
    files::Directory directory;
    OriginalPaths paths;
    paths.canonical=directory.path/"absent-canonical";
    paths.scope=directory.path/"absent-scope.json";
    paths.member=directory.path/"absent-member.key";
    const auto result=Inspect([&] { (void)ReadCanonical(paths,"not the original member"); return std::string{}; });
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error,"Source member bytes differ from authority");
    EXPECT_TRUE(std::filesystem::is_empty(directory.path));
}
TEST(OriginalSourceIO, RepeatedReadsReauthenticateReplacedBytesWithoutRetainingAValue) {
    files::Directory directory;
    const auto path=directory.path/"member.key";
    const auto hash=output::Sha256("abc");
    files::Overwrite(path,"abc"); EXPECT_TRUE(CompareRead(path,3,hash).accepted);
    files::Overwrite(path,"abd"); EXPECT_FALSE(CompareRead(path,3,hash).accepted);
    files::Overwrite(path,"abc"); EXPECT_TRUE(CompareRead(path,3,hash).accepted);
}
} // namespace crash::cases::vehicle_run::detail::test
