#pragma once
#include "../Internal.h"
#include "output/full_shell/tests/TestSupport.h"

namespace crash::output::full_shell::activity::test {
namespace base=full_shell::test;
inline ActivityInput Input(const Context& c,const FrameStamp& s,const std::vector<std::uint8_t>& values) {
    return {c.identity(),c.point_layout_sha256(),s,values.data(),values.size()};
}
inline Context Sized(std::size_t count,std::size_t nodes=4) {
    std::vector<ParentPoints> parents;parents.reserve(count);
    for(std::size_t i=0;i<count;++i)
        parents.push_back({1000000+i,2000000+i%867,2,1,3,PlasticField::NativeEquivalentPlasticStrain});
    return Context::Create(base::Id(),nodes,parents.data(),parents.size(),.125);
}
inline Document Metadata(const base::Directory& dir,const RecordFile& record) {
    return array_json::Parse(ReadBounded(dir.path/record.file,MetadataByteCap),MetadataByteCap);
}
inline RecordFile Rewrite(base::Directory& dir,const RecordFile& record,const Document& doc) {
    return base::Rewrite(dir.path,record,doc);
}
inline void WordBytes(base::Directory& dir,Document& doc,std::string bytes) {
    const auto file=array_json::Text(doc["activity"]["file"]);base::Overwrite(dir.path/file,bytes);
    const auto hash=Sha256(bytes);doc["activity"]["sha256"].SetString(hash.c_str(),doc.GetAllocator());
}
} // namespace crash::output::full_shell::activity::test
