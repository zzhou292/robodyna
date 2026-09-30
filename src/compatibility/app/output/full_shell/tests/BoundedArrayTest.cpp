#include "TestSupport.h"
#include <cstdlib>
#include <limits>

namespace crash::output::full_shell::test {
namespace ar=arrays;
TEST(BoundedArrays,ExactValuesEndianAndCanonicalDescriptors) {
    Directory dir;const double values[]{0.,-0.,std::numeric_limits<double>::denorm_min(),1.2345678901234567};
    const ar::Layout layout{ar::Scalar::Float64,2,2,{"a","b"}};
    const auto d=ar::Write(dir.path,"values.bin",layout,values,4);
    const auto raw=ReadBounded(dir.path/d.file,32);
    for(std::size_t i=0;i<4;++i)for(unsigned j=0;j<8;++j)
        EXPECT_EQ(static_cast<unsigned char>(raw[8*i+j]),(Bits(values[i])>>(8*j))&255);
    const auto back=ar::Read<double>(dir.path,ar::ParseDescriptor(ar::DescriptorDocument(d)));
    for(std::size_t i=0;i<4;++i)EXPECT_EQ(Bits(values[i]),Bits(back[i]));
    const std::uint64_t ids[]{UINT64_MAX,UINT64_C(9007199254740993)};
    const auto id=ar::Write(dir.path,"ids.bin",{ar::Scalar::UInt64,2,1,{}},ids,2);
    EXPECT_EQ(ar::Read<std::uint64_t>(dir.path,id),(std::vector<std::uint64_t>{ids[0],ids[1]}));
    EXPECT_THROW(ar::Read<double>(dir.path,id),std::exception);
}
TEST(BoundedArrays,PreflightBeforeBorrowedReadsAndExactByteBoundaries) {
    const auto* poison=reinterpret_cast<const double*>(1);
    EXPECT_THROW(ar::Encode({ar::Scalar::Float64,UINT64_MAX,64,{}},poison,1),std::exception);
    EXPECT_THROW(ar::Encode({ar::Scalar::Float64,3,1,{}},poison,3,{16,100,64}),std::exception);
    EXPECT_EQ(ar::ByteCount({ar::Scalar::UInt16,kArtifactFileCap/2,1,{}}),kArtifactFileCap);
    EXPECT_THROW(ar::ByteCount({ar::Scalar::UInt16,kArtifactFileCap/2+1,1,{}}),std::exception);
    EXPECT_THROW(ar::ByteCount({ar::Scalar::Float64,1,2,{"same","same"}}),std::exception);
    EXPECT_THROW(ar::ByteCount({ar::Scalar::Float64,1,1,{std::string("a\0b",3)}}),std::exception);
    Directory dir;const double nan=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ar::Write(dir.path,"nan.bin",{ar::Scalar::Float64,1,1,{}},&nan,1),std::exception);
    EXPECT_TRUE(std::filesystem::is_empty(dir.path));
}
TEST(BoundedArrays,RejectsTruncationTrailingHashNonfiniteAndUnsafePaths) {
    Directory dir;const double values[]{1,2,3};
    const auto d=ar::Write(dir.path,"good.bin",{ar::Scalar::Float64,3,1,{}},values,3);
    const auto original=ReadBounded(dir.path/d.file,24);
    for(const auto& bad:{original.substr(0,23),original+"x",std::string(24,'z')}) {
        Overwrite(dir.path/d.file,bad);EXPECT_THROW(ar::Read<double>(dir.path,d),std::exception);
    }
    auto bytes=original;const auto inf=UINT64_C(0x7ff0000000000000);
    for(unsigned j=0;j<8;++j)bytes[16+j]=static_cast<char>(inf>>(8*j));
    auto rehashed=d;rehashed.sha256=Sha256(bytes);Overwrite(dir.path/d.file,bytes);
    EXPECT_THROW(ar::Read<double>(dir.path,rehashed),std::exception);
    for(const auto* name:{"../x","/tmp/x","a//b","a/../x","./x"})
        EXPECT_THROW(ar::CheckedPath(dir.path,name,false),std::exception);
    std::filesystem::create_symlink("good.bin",dir.path/"alias.bin");
    auto alias=d;alias.file="alias.bin";EXPECT_THROW(ar::Read<double>(dir.path,alias),std::exception);
    std::filesystem::create_symlink("missing.bin",dir.path/"dangling.bin");
    EXPECT_THROW(ar::Write(dir.path,"dangling.bin",d.layout,values,3),std::exception);
    auto doc=ar::DescriptorDocument(d);doc.AddMember("bytes",24,doc.GetAllocator());
    EXPECT_THROW(ar::ParseDescriptor(doc),std::exception);
}
template<class T> void CopyCanonical(const std::filesystem::path& path,const ar::Descriptor& d) {
    const auto values=ar::Read<T>(path,d);
    auto copy=ar::Write(path,"cpp-"+std::filesystem::path(d.file).filename().string(),d.layout,values.data(),values.size());
    EXPECT_EQ(d.sha256,copy.sha256);EXPECT_EQ(d.bytes,copy.bytes);
    WriteJson(path/("cpp-"+std::filesystem::path(d.file).stem().string()+".json"),ar::DescriptorDocument(copy));
}
TEST(BoundedArrays,CanonicalImporterInteroperability) {
    const auto* root=std::getenv("ROBO_DYNA_CANONICAL_ARRAY_FIXTURE");ASSERT_NE(root,nullptr);
    const std::filesystem::path path=root;const auto doc=array_json::Parse(ReadBounded(path/"manifest.json",65536),65536);
    ASSERT_TRUE(doc.HasMember("arrays"));ASSERT_EQ(doc["arrays"].MemberCount(),6u);
    for(auto it=doc["arrays"].MemberBegin();it!=doc["arrays"].MemberEnd();++it) {
        const auto d=ar::ParseDescriptor(it->value);
        switch(d.layout.scalar) {
            case ar::Scalar::UInt16:CopyCanonical<std::uint16_t>(path,d);break;
            case ar::Scalar::UInt32:CopyCanonical<std::uint32_t>(path,d);break;
            case ar::Scalar::UInt64:CopyCanonical<std::uint64_t>(path,d);break;
            case ar::Scalar::Int32:CopyCanonical<std::int32_t>(path,d);break;
            case ar::Scalar::Float64:CopyCanonical<double>(path,d);break;
        }
    }
}
} // namespace crash::output::full_shell::test
