#include "ComponentContext.h"
#include <array>

namespace crash::output::assembly::binary {
namespace {
void AppendArray(std::string& digest_input,const char* name,std::size_t columns,
                 const std::vector<std::uint64_t>& values) {
    const arrays::Layout layout{arrays::Scalar::UInt64,values.size()/columns,columns,{}};
    const auto bytes=arrays::Encode(layout,values.data(),values.size());
    // Fixed order, named layouts and fixed-length hashes make the domain
    // unambiguous without paths, native structs, run IDs or host endianness.
    const std::array<std::uint64_t,3> shape{layout.rows,columns,bytes.size()};
    digest_input.append(name).push_back('\0');
    digest_input.append(arrays::Dtype(layout.scalar)).push_back('\0');
    digest_input+=arrays::Encode({arrays::Scalar::UInt64,1,3,{}},shape.data(),shape.size());
    digest_input+=Sha256(bytes);
}
}
std::string ComponentMappingDigest(const SourceAssemblySurface& surface) {
    CheckWallSourceSchema(surface.source().data().schema);
    const auto& binding=surface.binding();const auto& parents=surface.parents();
    Require(!parents.empty()&&parents.size()<=1024&&binding.vertices.size()<=2048&&
        binding.triangles.size()<=2048,"Component mapping exceeds its existing source surface scope");
    std::string input=MappingDomain;input.push_back('\0');
    std::vector<std::uint64_t> values;
    values.reserve(13*parents.size());
    for(const auto& node:binding.vertices) {
        values.push_back(node.tl_node);values.push_back(node.source.node);
    }
    AppendArray(input,"nodes",2,values);values.clear();
    for(const auto& p:parents) {
        for(std::uint64_t value:std::array<std::uint64_t,13>{p.source_index,p.element,p.part,p.material,p.section,p.curve,
            p.source_elform,p.family==source::ShellFamily::Qeph?QephFamily:T3Family,
            p.family_index,std::uint64_t(3),std::uint64_t(records::PlasticField::NativeEquivalentPlasticStrain),
            p.first_triangle,p.triangle_count})values.push_back(value);
    }
    AppendArray(input,"parents",13,values);values.clear();
    for(const auto& t:binding.triangles) {
        for(std::uint64_t value:std::array<std::uint64_t,7>{std::uint64_t(t.vertices[0]),std::uint64_t(t.vertices[1]),
            std::uint64_t(t.vertices[2]),t.element,t.part,std::uint64_t(t.local_face),
            std::uint64_t(t.subtriangle)})values.push_back(value);
    }
    AppendArray(input,"triangles",7,values);
    return Sha256(input);
}
} // namespace crash::output::assembly::binary
