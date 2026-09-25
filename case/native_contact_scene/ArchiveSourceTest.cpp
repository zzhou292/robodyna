#include "ArchiveSource.h"
#include "output/full_shell/static_bundle/SourceBundle.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
#include <algorithm>
#include "output/physical_frames/NativeSourceMapping.h"
#include "output/full_shell/static_bundle/MappingArrays.h"
namespace crash::cases::native_scene {
namespace records=output::full_shell;
namespace source=records::source;
namespace ft=records::test;
using namespace output;
namespace {
PhysicalSource Physical() {
    const auto* raw=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");if(!raw)throw std::runtime_error("Explicit scene fixture required");
    const std::filesystem::path path(raw);
    return PhysicalSource::Prepare(modelio::native_scene::DeclaredSource::Read(path,Sha256(ReadBounded(path,4u<<20))),771);
}
source::BundleRequest Request(const source::PreparedSourceMapping& mapping) {
    source::BundleRequest request;auto& a=request.archive;
    a.nodes=mapping.nodes();a.parents=mapping.parents().size();a.plastic_points=36;
    a.frames=31;a.intervals=1000;a.fixed_dt=3e-7;a.requested_duration=.0003;
    a.static_files={{"manifest.json",1u<<20},{"frame-index.json",1u<<20},{"configuration.json",1u<<20}};
    return request;
}
}
TEST(NativeSceneArchiveSource, DeclaredShellOnlySourceAndEmptyFamiliesRoundTripExactly) {
    ft::Directory original,repacked;const auto physical=Physical();
    const auto archive=ArchiveSource::Write(physical,original.path);const auto& mapping=archive.mapping();
    ASSERT_EQ(mapping.nodes(),18u);ASSERT_EQ(mapping.parents().size(),12u);ASSERT_EQ(mapping.triangles(),16u);
    EXPECT_EQ(array_json::Text(array_json::Parse(mapping.source().data().canonical_bytes,1u<<20)["schema"]),source::DeclaredCanonicalSchema);
    EXPECT_EQ(mapping.parents()[0].source_element,1u);EXPECT_EQ(mapping.parents()[8].source_element,9u);
    EXPECT_EQ(mapping.parents()[0].source_elform,24u); // Explicit native ISHELL scheme, not a DYNA card.
    const auto bundle=source::PreparedSourceBundle::Prepare(mapping,Request(mapping));
    std::filesystem::create_directory(repacked.path/"arrays");
    const auto descriptor=source::WriteSourceBundle(repacked.path,bundle);
    const auto restored=source::ReadSourceBundle(repacked.path,descriptor,mapping.source().data().inputs,mapping.digest());
    EXPECT_EQ(restored.digest(),mapping.digest());EXPECT_EQ(restored.nodes(),mapping.nodes());
    std::size_t empty=0;
    for(const auto& a:restored.source().data().arrays)if(a.descriptor.layout.rows==0) {
        ++empty;EXPECT_EQ(a.descriptor.bytes,0u);EXPECT_EQ(a.descriptor.sha256,Sha256({}));EXPECT_TRUE(a.bytes.empty());
        EXPECT_EQ(std::filesystem::file_size(repacked.path/a.descriptor.file),0u);
        const auto reserve=bundle.reservations();
        const auto found=std::find_if(reserve.begin(),reserve.end(),[&](const auto& f){return f.file==a.descriptor.file;});
        ASSERT_NE(found,reserve.end());EXPECT_EQ(found->bytes,1u); // Positive upper-bound reservation only.
    }
    EXPECT_EQ(empty,8u);
    EXPECT_THROW(ArchiveSource::Write(physical,original.path),std::exception);
}
TEST(NativeSceneArchiveSource, CorruptedEmptyHashOrActualFileNeverBecomesTypedSource) {
    ft::Directory original,repacked;const auto archive=ArchiveSource::Write(Physical(),original.path);
    const auto& mapping=archive.mapping();const auto bundle=source::PreparedSourceBundle::Prepare(mapping,Request(mapping));
    std::filesystem::create_directory(repacked.path/"arrays");const auto descriptor=source::WriteSourceBundle(repacked.path,bundle);
    const auto pristine=ReadBounded(repacked.path/descriptor.file,source::BundleMetadataByteCap);
    auto corrupt=pristine;const auto index=corrupt.find(Sha256({}));ASSERT_NE(index,std::string::npos);
    corrupt.replace(index,64,std::string(64,'0'));ft::Overwrite(repacked.path/descriptor.file,corrupt);
    const records::RecordFile changed{descriptor.file,Sha256(corrupt),corrupt.size()};
    EXPECT_THROW(source::ReadSourceBundle(repacked.path,changed,mapping.source().data().inputs,mapping.digest()),std::exception);
    ft::Overwrite(repacked.path/descriptor.file,pristine);
    const auto& array=source::FindArray(mapping.source().data(),"solids_records");
    ft::Overwrite(repacked.path/array.descriptor.file,"x");
    EXPECT_THROW(source::ReadSourceBundle(repacked.path,descriptor,mapping.source().data().inputs,mapping.digest()),std::exception);
}
TEST(NativeSceneArchiveSource, NonzeroDeclaredArrayCannotUseAnEmptyPayload) {
    ft::Directory directory;const auto archive=ArchiveSource::Write(Physical(),directory.path);
    auto input=archive.mapping().source().data().inputs;
    auto canonical=array_json::Parse(ReadBounded(directory.path/input.canonical_manifest.file,1u<<20),1u<<20);
    canonical["arrays"]["node_ids"]["bytes"].SetUint64(0);
    canonical["arrays"]["node_ids"]["sha256"].SetString(Sha256({}).c_str(),canonical.GetAllocator());
    input.canonical_manifest=ft::Rewrite(directory.path,input.canonical_manifest,canonical);
    auto scope=array_json::Parse(ReadBounded(directory.path/input.scope_report.file,1u<<20),1u<<20);
    scope["canonical_sha256"].SetString(input.canonical_manifest.sha256.c_str(),scope.GetAllocator());
    input.scope_report=ft::Rewrite(directory.path,input.scope_report,scope);
    EXPECT_THROW(source::CanonicalSource::Read(input),std::exception);
}
TEST(NativeSceneArchiveSource, EqualIdsCannotBindDifferentReferenceCoordinatesOrOrderedConnectivity) {
    const auto physical=Physical();
    for(bool connectivity:{false,true}) {
        ft::Directory directory;const auto archive=ArchiveSource::Write(physical,directory.path);
        const auto& original=archive.mapping();
        auto input=original.source().data().inputs;
        auto canonical=array_json::Parse(original.source().data().canonical_bytes,1u<<20);
        const auto replace_array=[&](const char* name,const auto& values) {
            const auto& a=source::FindArray(original.source().data(),name);
            const auto bytes=arrays::Encode(a.descriptor.layout,values.data(),values.size());
            ft::Overwrite(directory.path/a.descriptor.file,bytes);
            canonical["arrays"][name]["sha256"].SetString(Sha256(bytes).c_str(),canonical.GetAllocator());
        };
        if(connectivity) {
            const auto& indices=source::FindArray(original.source().data(),"shells_node_indices");
            const auto& records_array=source::FindArray(original.source().data(),"shells_records");
            auto local=arrays::Decode<std::uint32_t>(indices.descriptor,indices.bytes);
            auto ids=arrays::Decode<std::uint64_t>(records_array.descriptor,records_array.bytes);
            // A cyclic Q4 permutation preserves geometry/IDs but changes native order.
            std::rotate(local.begin()+32,local.begin()+33,local.begin()+36);
            std::rotate(ids.begin()+50,ids.begin()+51,ids.begin()+54);
            replace_array("shells_node_indices",local);replace_array("shells_records",ids);
        } else {
            const auto& a=source::FindArray(original.source().data(),"node_positions");
            auto xyz=arrays::Decode<double>(a.descriptor,a.bytes);xyz[2]=.001;
            replace_array("node_positions",xyz);
        }
        input.canonical_manifest=ft::Rewrite(directory.path,input.canonical_manifest,canonical);
        auto scope=array_json::Parse(original.source().data().scope_bytes,1u<<20);
        scope["canonical_sha256"].SetString(input.canonical_manifest.sha256.c_str(),scope.GetAllocator());
        input.scope_report=ft::Rewrite(directory.path,input.scope_report,scope);
        const auto altered=source::CanonicalSource::Read(input);
        const auto& node_array=original.arrays()[source::detail::NodeCanonical];
        const auto nodes=arrays::Decode<std::uint32_t>(node_array.descriptor,node_array.bytes);
        const auto& ref_array=original.arrays()[source::detail::ParentReference];
        const auto refs=arrays::Decode<std::uint32_t>(ref_array.descriptor,ref_array.bytes);
        std::vector<source::NativeParent> parents;
        for(std::size_t j=0;j<original.parents().size();++j)
            parents.push_back({refs[3*j],refs[3*j+1],refs[3*j+2],original.parents()[j].native_points,original.parents()[j].plastic});
        const auto foreign=source::PreparedSourceMapping::Prepare(altered,{nodes.data(),nodes.size(),parents.data(),parents.size()});
        EXPECT_THROW(physical_frames::detail::BindNativeSource(foreign,physical.physical(),64u<<20),std::exception);
        const auto bound=physical_frames::detail::BindNativeSource(original,physical.physical(),64u<<20);
        EXPECT_EQ(bound.nodes,archive.physical_nodes());EXPECT_EQ(bound.parents.size(),12u);
        const auto bytes=physical_frames::detail::NativeMappingBytes(original,64u<<20);
        EXPECT_NO_THROW(physical_frames::detail::BindNativeSource(original,physical.physical(),bytes));
        EXPECT_THROW(physical_frames::detail::BindNativeSource(original,physical.physical(),bytes-1),std::exception);
    }
}

}
