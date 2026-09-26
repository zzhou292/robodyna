#include "ArchiveSource.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/static_bundle/CanonicalSpecs.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <algorithm>
#include <numeric>
namespace crash::cases::native_scene {
namespace records=output::full_shell;
namespace source=records::source;
namespace arrays=output::arrays;
using namespace output;
struct ArchiveSource::Data {
    Data(const PhysicalSource& p,source::PreparedSourceMapping m):physical(p),mapping(std::move(m)){}
    PhysicalSource physical;
    source::PreparedSourceMapping mapping;
    std::vector<std::uint32_t> nodes;
    std::vector<physical_frames::ParentField> parents;
};
namespace {
std::string EncodeDocument(const Document& d) {
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    Require(d.Accept(writer)&&buffer.GetSize()<=1u<<20,"Declared source metadata exceeds bounded cap");
    return {buffer.GetString(),buffer.GetSize()};
}
Document MemberDocument(const source::SourceInputs& inputs) {
    Document d;d.SetObject();String(d,"file",inputs.source_member.file);String(d,"sha256",inputs.source_member.sha256);
    Integer(d,"bytes",inputs.source_member.bytes);return d;
}
Document UnitDocument() {
    Document d;d.SetObject();String(d,"mass","tonne");String(d,"length","mm");String(d,"time","s");
    Number(d,"mass_to_kg",1000);Number(d,"length_to_m",.001);Number(d,"time_to_s",1);return d;
}
std::string IdHash(const std::vector<std::uint64_t>& values) {
    return Sha256(arrays::Encode({arrays::Scalar::UInt64,values.size(),1,{}},values.data(),values.size()));
}
template<class T> void Array(std::array<source::NamedArray,source::CanonicalArrayCount>& output,
    std::size_t index,std::size_t rows,const std::vector<T>& values) {
    const auto& spec=source::detail::CanonicalSpecs()[index];auto& a=output[index];a.name=spec.name;
    arrays::Layout layout{spec.scalar,rows,spec.columns,source::detail::FieldNames(spec.fields)};
    a.bytes=arrays::Encode(layout,values.data(),values.size());
    a.descriptor={"arrays/declared-"+a.name+".bin",layout,a.bytes.size(),Sha256(a.bytes)};
}
}
ArchiveSource ArchiveSource::Write(const PhysicalSource& physical,const std::filesystem::path& root) {
    const auto& declared=physical.declared().data();const auto& binding=physical.physical();
    Require(binding.prepared()&&binding.domain()->node_count()==declared.nodes.size(),"Declared archive source differs from physical extent");
    source::SourceInputs inputs;inputs.canonical_root=inputs.scope_root=inputs.member_root=root;
    inputs.tire_policy="retain_all";inputs.units={"tonne","mm","s",1000,.001,1};
    inputs.source_member={"declared-scene-source.json",declared.definition_sha256,declared.definition_bytes.size()};
    std::array<source::NamedArray,source::CanonicalArrayCount> arrays_value;
    std::vector<std::uint64_t> ids,parents;
    std::vector<std::uint32_t> connectivity;
    std::vector<double> positions;
    for(std::size_t i=0;i<declared.nodes.size();++i) {
        const auto& n=binding.domain()->nodes()[i];Require(n.source_id==declared.nodes[i].id,"Declared archive source-node identity differs");
        ids.push_back(n.source_id);positions.insert(positions.end(),{n.position.x,n.position.y,n.position.z});
    }
    std::vector<std::uint64_t> parent_ids;
    for(const auto* family:{&declared.wall,&declared.patch})for(const auto& p:*family) {
        parent_ids.push_back(p.id);parents.push_back(p.id);parents.push_back(p.part);
        for(auto node:p.nodes){parents.push_back(declared.nodes[node].id);connectivity.push_back(node);}
    }
    const auto n=ids.size(),e=parent_ids.size();
    Array(arrays_value,0,n,ids);Array(arrays_value,1,n,positions);
    Array(arrays_value,2,n,std::vector<std::int32_t>(2*n));Array(arrays_value,3,n,std::vector<std::uint16_t>(n));
    Array(arrays_value,4,n,std::vector<std::uint32_t>(n));Array(arrays_value,5,e,parents);Array(arrays_value,6,e,connectivity);
    Array(arrays_value,7,e,std::vector<std::uint32_t>(e));Array(arrays_value,8,e,std::vector<std::uint16_t>(e));
    for(std::size_t i:{9u,13u})Array(arrays_value,i,0,std::vector<std::uint64_t>{});
    for(std::size_t i:{10u,11u,14u,15u})Array(arrays_value,i,0,std::vector<std::uint32_t>{});
    for(std::size_t i:{12u,16u})Array(arrays_value,i,0,std::vector<std::uint16_t>{});
    Document canonical;canonical.SetObject();String(canonical,"schema",source::DeclaredCanonicalSchema);
    String(canonical,"purpose","declared_shell_geometry_only_not_simulation_or_restart");
    String(canonical,"source_format",declared.definition_schema);
    String(canonical,"formulation_scheme","openradioss_property_type1_ishell");
    String(canonical,"keyword_annotations","not_applicable_zero_channels");
    array_json::Child(canonical,"member",MemberDocument(inputs));array_json::Child(canonical,"units",UnitDocument());
    Document counts;counts.SetObject();Integer(counts,"nodes",n);Integer(counts,"shells",e);Integer(counts,"solids",0);Integer(counts,"beams",0);
    array_json::Child(canonical,"counts",counts);
    Value materials(rapidjson::kArrayType),sections(rapidjson::kArrayType),parts(rapidjson::kArrayType);
    Document material;material.SetObject();Integer(material,"source_material_id",1);String(material,"source_law","LAW44");
    materials.PushBack(Value(material,canonical.GetAllocator()),canonical.GetAllocator());
    Document section;section.SetObject();Integer(section,"source_section_id",1);Integer(section,"source_formulation_code",24);Integer(section,"source_nip",3);
    sections.PushBack(Value(section,canonical.GetAllocator()),canonical.GetAllocator());
    for(unsigned pid:{1u,2u}) {
        Document part;part.SetObject();Integer(part,"source_part_id",pid);Integer(part,"source_material_id",1);Integer(part,"source_section_id",1);
        parts.PushBack(Value(part,canonical.GetAllocator()),canonical.GetAllocator());
    }
    canonical.AddMember("materials",materials,canonical.GetAllocator());canonical.AddMember("sections",sections,canonical.GetAllocator());canonical.AddMember("parts",parts,canonical.GetAllocator());
    Document descriptors;descriptors.SetObject();
    for(const auto& a:arrays_value)array_json::Child(descriptors,a.name.c_str(),arrays::DescriptorDocument(a.descriptor));
    array_json::Child(canonical,"arrays",descriptors);const auto canonical_bytes=EncodeDocument(canonical);
    inputs.canonical_manifest={"declared-canonical.json",Sha256(canonical_bytes),canonical_bytes.size()};
    Document scope;scope.SetObject();String(scope,"schema",source::DeclaredScopeSchema);
    String(scope,"purpose","complete_declared_shell_source_only");String(scope,"selection","all_declared_shells");
    String(scope,"canonical_sha256",inputs.canonical_manifest.sha256);
    array_json::Child(scope,"member",MemberDocument(inputs));array_json::Child(scope,"units",UnitDocument());
    Document coverage;coverage.SetObject();Integer(coverage,"nodes",n);Integer(coverage,"shells",e);
    Integer(coverage,"q4",declared.patch.size());Integer(coverage,"t3",declared.wall.size());
    String(coverage,"retained_shell_ids_sha256",IdHash(parent_ids));String(coverage,"excluded_shell_ids_sha256",IdHash({}));
    auto sorted=ids;std::sort(sorted.begin(),sorted.end());String(coverage,"selected_node_ids_sha256",IdHash(sorted));
    array_json::Child(scope,"coverage",coverage);const auto scope_bytes=EncodeDocument(scope);
    inputs.scope_report={"declared-scope.json",Sha256(scope_bytes),scope_bytes.size()};
    std::size_t bytes=canonical_bytes.size()+scope_bytes.size()+declared.definition_bytes.size();
    Require(bytes<=64u<<20,"Declared static metadata exceeds cap");
    for(const auto& a:arrays_value){Require(a.bytes.size()<=64u*1024*1024-bytes,"Declared static payload exceeds cap");bytes+=a.bytes.size();}
    Require(bytes<=64u<<20,"Declared static payload exceeds cap");
    std::vector<std::uint32_t> nodes(n);std::iota(nodes.begin(),nodes.end(),0);
    std::vector<source::NativeParent> native;std::vector<physical_frames::ParentField> fields;
    const auto parent=[&](std::size_t canonical_parent,tl::fea::ShellBindingFamily family,std::size_t index) {
        const auto* role=binding.execution()->parent(family,index);
        const auto& row=family==tl::fea::ShellBindingFamily::T3?declared.wall[index]:declared.patch[index];
        const bool rigid=declared.rigid_patch&&row.part==declared.rigid_patch->source_part_id;
        const auto law=rigid?tl::fea::ShellSectionLaw::RigidSkin:tl::fea::ShellSectionLaw::LayeredLaw44Nip3;
        Require(role&&role->source.source_parent_id==row.id&&role->source.source_part_id==row.part&&
            role->law==law&&role->material_points==(rigid?0u:3u),
            "Declared archive source/material role differs from physical execution");
        const auto wire_family=physical_frames::Family(family);
        native.push_back({std::uint32_t(canonical_parent),wire_family,std::uint32_t(index),role->material_points,physical_frames::Plasticity(role->law)});
        fields.push_back({wire_family,std::uint32_t(index),role->law});
    };
    for(std::size_t i=0;i<declared.wall.size();++i)parent(i,tl::fea::ShellBindingFamily::T3,i);
    for(std::size_t i=0;i<declared.patch.size();++i)parent(declared.wall.size()+i,tl::fea::ShellBindingFamily::Qeph,i);
    // All representation and size checks precede the first output mutation.
    Require(std::filesystem::symlink_status(root).type()==std::filesystem::file_type::directory&&std::filesystem::is_empty(root),"Declared source output requires a real empty directory");
    std::filesystem::create_directory(root/"arrays");
    WriteBytes(root/inputs.source_member.file,declared.definition_bytes);
    WriteBytes(root/inputs.canonical_manifest.file,canonical_bytes);WriteBytes(root/inputs.scope_report.file,scope_bytes);
    for(const auto& a:arrays_value)arrays::WriteBytes(root,a.descriptor.file,a.descriptor.layout,a.bytes);
    auto canonical_source=source::CanonicalSource::Read(inputs);
    auto mapping=source::PreparedSourceMapping::Prepare(canonical_source,{nodes.data(),nodes.size(),native.data(),native.size()});
    auto result=std::make_shared<Data>(physical,std::move(mapping));result->nodes=std::move(nodes);result->parents=std::move(fields);
    return ArchiveSource(std::move(result));
}
const PhysicalSource& ArchiveSource::physical_source() const noexcept{return data_->physical;}
const source::PreparedSourceMapping& ArchiveSource::mapping() const noexcept{return data_->mapping;}
const std::vector<std::uint32_t>& ArchiveSource::physical_nodes() const noexcept{return data_->nodes;}
const std::vector<physical_frames::ParentField>& ArchiveSource::parents() const noexcept{return data_->parents;}
}
