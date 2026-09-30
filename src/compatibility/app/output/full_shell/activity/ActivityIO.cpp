#include "Internal.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::output::full_shell::activity {
namespace detail {
std::string EncodeDocument(const Document& doc) {
    rapidjson::StringBuffer b;rapidjson::Writer<rapidjson::StringBuffer> w(b);
    Require(doc.Accept(w)&&b.GetSize()<=MetadataByteCap,"Activity metadata exceeds capacity");return {b.GetString(),b.GetSize()};
}
Document ReadDocument(const std::filesystem::path& root,const RecordFile& file) {
    arrays::CheckHash(file.sha256);Require(file.bytes&&file.bytes<=MetadataByteCap,"Invalid activity metadata size");
    const auto path=arrays::CheckedPath(root,file.file,true);
    Require(std::filesystem::file_size(path)==file.bytes,"Activity metadata file size mismatch");
    const auto bytes=ReadBounded(path,file.bytes);
    Require(bytes.size()==file.bytes&&Sha256(bytes)==file.sha256,"Activity metadata hash/size mismatch");
    return array_json::Parse(bytes,MetadataByteCap);
}
}
RecordFile WriteActivity(const std::filesystem::path& root,const std::string& stem,const ActivityRecord& record) {
    arrays::CheckRelativeName(stem);Require(stem.find('/')==std::string::npos&&stem.size()<=128,"Invalid activity stem");
    const auto& c=record.context();const auto layout=detail::Layout(c.parents().size());
    const auto bytes=arrays::Encode(layout,record.words().data(),record.words().size(),c.limits().arrays);
    const arrays::Descriptor array{stem+".activity.bin",layout,bytes.size(),Sha256(bytes)};
    const auto json=detail::EncodeDocument(detail::Frame(record,array));
    const RecordFile result{stem+".activity.json",Sha256(json),json.size()};
    const auto value_path=arrays::CheckedPath(root,array.file,false),metadata_path=arrays::CheckedPath(root,result.file,false);
    output::WriteBytes(value_path,bytes);output::WriteBytes(metadata_path,json);return result;
}
ActivityRecord ReadActivity(const std::filesystem::path& root,const Context& c,const RecordFile& file,
                            const FrameStamp& expected,Limits limits) {
    detail::Preflight(c,limits);CheckStamp(c,expected);
    const auto doc=detail::ReadDocument(root,file);const auto array=detail::ParseFrame(c,doc,expected);
    Require(array.file!=file.file,"Activity metadata aliases packed values");
    auto words=arrays::Read<std::uint64_t>(root,array,c.limits().arrays);
    return ActivityRecord::FromWords(c,expected,std::move(words));
}
RecordFile WriteDeclaration(const std::filesystem::path& root,const std::string& file,const Context& c,Limits limits) {
    detail::Preflight(c,limits);const auto bytes=detail::EncodeDocument(detail::Declaration(c));
    const RecordFile result{file,Sha256(bytes),bytes.size()};const auto path=arrays::CheckedPath(root,file,false);
    output::WriteBytes(path,bytes);return result;
}
void ReadDeclaration(const std::filesystem::path& root,const Context& c,const RecordFile& file,Limits limits) {
    detail::Preflight(c,limits);const auto doc=detail::ReadDocument(root,file);detail::CheckDeclaration(c,doc);
}
} // namespace crash::output::full_shell::activity
