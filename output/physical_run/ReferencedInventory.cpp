#include "Metadata.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/static_bundle/SourceBundle.h"
#include "output/full_shell/activity/Internal.h"
#include <map>

namespace crash::output::physical_run {
std::vector<records::RecordFile> ReferencedSampleFiles(const std::filesystem::path& root,
    const records::Context& context,const records::RecordFile& source_bundle,
    const records::RecordFile& activity_declaration,const std::vector<FrameFiles>& frames,
    const std::optional<WallReceipt>& wall) {
    std::map<std::string,records::RecordFile> expected;
    const auto add=[&](const records::RecordFile& file) {
        FileDocument(file);
        Require(expected.emplace(file.file,file).second,"Physical records share a file owner");
    };
    const auto array=[&](const arrays::Descriptor& file) {add({file.file,file.sha256,file.bytes});};
    add(source_bundle);add(activity_declaration);
    if(wall) {
        WallDocument(*wall);
        for(const auto& file:wall->files)add(file);
    }
    const auto source=array_json::Parse(ReadFile(root,source_bundle,records::source::BundleMetadataByteCap),
        records::source::BundleMetadataByteCap);
    Require(source.HasMember("files") && source["files"].IsArray(),"Physical source inventory is missing");
    for(const auto& file:source["files"].GetArray())add(ReadFileRecord(file));
    for(const auto& frame:frames) {
        add(frame.frame);add(frame.activity);
        const auto values=records::ParseFrameDocument(context,
            array_json::Parse(ReadFile(root,frame.frame,MetadataCap),MetadataCap));
        array(values.positions);array(values.plastic);
        const auto activity=records::activity::detail::ParseFrame(context,
            array_json::Parse(ReadFile(root,frame.activity,MetadataCap),MetadataCap),frame.stamp);
        array(activity);
    }
    std::vector<records::RecordFile> files;
    for(const auto& row:expected)files.push_back(row.second);
    return files;
}
void CheckReferencedInventory(const std::filesystem::path& root,const records::Context& context,
    const Index& index,const Manifest& manifest) {
    std::map<std::string,records::RecordFile> expected;
    const auto add=[&](const records::RecordFile& file) {
        FileDocument(file);
        Require(expected.emplace(file.file,file).second,"Physical records share a file owner");
    };
    for(const auto& file:ReferencedSampleFiles(root,context,manifest.source,manifest.activity_declaration,
        index.frames,manifest.wall))add(file);
    if(manifest.environment) {
        EnvironmentDocument(*manifest.environment);
        for(const auto& file:manifest.environment->files)add(file);
    }
    add(manifest.configuration);add(manifest.index);
    for(const auto& segment:index.segments) {
        add({segment.integers.file,segment.integers.sha256,segment.integers.bytes});
        add({segment.reals.file,segment.reals.sha256,segment.reals.bytes});
    }
    Require(expected.size()==manifest.inventory.size(),"Physical archive contains unreferenced files");
    for(const auto& file:manifest.inventory) {
        const auto found=expected.find(file.file);
        Require(found!=expected.end() && found->second.sha256==file.sha256 && found->second.bytes==file.bytes,
            "Physical inventory differs from its typed record owner");
    }
}
} // namespace crash::output::physical_run
