#include "SurfaceBindingFields.h"
#include "chrono/SurfaceBinding.h"

namespace crash::output {
void AppendSurfaceBinding(Document& doc,const visual::Binding& b) {
    Value vertices(rapidjson::kArrayType),triangles(rapidjson::kArrayType);
    for(const auto& v:b.vertices) {
        Value item(rapidjson::kArrayType);
        for(std::uint64_t n:{std::uint64_t(v.tl_node),v.source.asset,v.source.instance,v.source.node})item.PushBack(Value().SetUint64(n),doc.GetAllocator());
        vertices.PushBack(item,doc.GetAllocator());
    }
    for(const auto& t:b.triangles) {
        Value item(rapidjson::kArrayType);
        for(std::uint64_t n:{std::uint64_t(t.vertices[0]),std::uint64_t(t.vertices[1]),std::uint64_t(t.vertices[2]),
                            t.asset,t.instance,t.element,t.part,std::uint64_t(t.local_face),std::uint64_t(t.subtriangle)})
            item.PushBack(Value().SetUint64(n),doc.GetAllocator());
        triangles.PushBack(item,doc.GetAllocator());
    }
    String(doc,"vertex_binding_columns","tl_node,asset,instance,source_node");
    String(doc,"triangle_binding_columns","v0,v1,v2,asset,instance,parent_element,part,local_face,subtriangle");
    doc.AddMember("vertex_binding",vertices,doc.GetAllocator()); doc.AddMember("triangle_binding",triangles,doc.GetAllocator());
}
} // namespace crash::output
