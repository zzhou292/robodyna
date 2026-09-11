#include "Metadata.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/static_bundle/SourceBundle.h"
#include "output/full_shell/activity/Internal.h"
#include <map>

namespace crash::output::physical_run {
void CheckReferencedInventory(const std::filesystem::path& root,const records::Context& context,
    const Index& index,const Manifest& manifest) {
    std::map<std::string,records::RecordFile> expected;
    const auto add=[&](const records::RecordFile& file) {
        FileDocument(file);
        Require(expected.emplace(file.file,file).second,"Physical records share a file owner");
    };
    const auto array=[&](const arrays::Descriptor& file) {add({file.file,file.sha256,file.bytes});};
    for(const auto* file:{&manifest.configuration,&manifest.index,&manifest.source,&manifest.activity_declaration})add(*file);
    if(manifest.wall) {
        WallDocument(*manifest.wall);
        for(const auto& file:manifest.wall->files)add(file);
    }
    const auto source=array_json::Parse(ReadFile(root,manifest.source,records::source::BundleMetadataByteCap),
        records::source::BundleMetadataByteCap);
    Require(source.HasMember("files") && source["files"].IsArray(),"Physical source inventory is missing");
    for(const auto& file:source["files"].GetArray())add(ReadFileRecord(file));
    for(const auto& segment:index.segments) {array(segment.integers);array(segment.reals);}
    for(const auto& frame:index.frames) {
        add(frame.frame);add(frame.activity);
        const auto values=records::ParseFrameDocument(context,
            array_json::Parse(ReadFile(root,frame.frame,MetadataCap),MetadataCap));
        array(values.positions);array(values.plastic);
        const auto activity=records::activity::detail::ParseFrame(context,
            array_json::Parse(ReadFile(root,frame.activity,MetadataCap),MetadataCap),frame.stamp);
        array(activity);
    }
    Require(expected.size()==manifest.inventory.size(),"Physical archive contains unreferenced files");
    for(const auto& file:manifest.inventory) {
        const auto found=expected.find(file.file);
        Require(found!=expected.end() && found->second.sha256==file.sha256 && found->second.bytes==file.bytes,
            "Physical inventory differs from its typed record owner");
    }
}
} // namespace crash::output::physical_run
