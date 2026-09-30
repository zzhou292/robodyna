#include "Fixture.h"
#include "output/BoundedArrayJson.h"
#include <iomanip>
#include <sstream>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

namespace crash::modelio::tied_shell::classification_test {
using namespace output;
std::string Card(std::initializer_list<unsigned> values, unsigned width) {
    std::ostringstream text;
    for (const auto value : values) text << std::setw(width) << value;
    return text.str();
}
namespace {
void Append(std::vector<SourceEvidence>& sources, const char* file, const char* keyword,
            std::initializer_list<std::string> cards) {
    SourceEvidence source;
    source.block.filename = file;
    source.block.keyword = keyword;
    source.block.first_line = sources.empty() ? 1 : sources.back().block.last_line + 1;
    source.block.raw_text = std::string(keyword) + "\n";
    auto line = source.block.first_line;
    for (const auto& card : cards) {
        source.cards.emplace_back(++line, card);
        source.block.raw_text += card + "\n";
    }
    source.block.last_line = line;
    source.block.sha256 = Sha256(source.block.raw_text);
    sources.push_back(std::move(source));
}
}
Fixture::Fixture() {
    declaration.slave_nodes = {{20,0,false,{}}, {30,1,false,{}}};
    auto& main = declaration.sources;
    Append(main, "yaris-coarse-v1l.key", "*CONSTRAINED_NODAL_RIGID_BODY", {Card({500,0,500,12})});
    Append(main, "yaris-coarse-v1l.key", "*CONSTRAINED_JOINT_SPHERICAL_ID", {Card({600}),Card({13,14,15,16,17})});
    Append(main, "yaris-coarse-v1l.key", "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE", {Card({2,1,2,2}),"",""});
    Append(main, "yaris-coarse-v1l.key", "*MAT_RIGID", {Card({2000})});
    declaration.contact_source = 2;
    GroupEvidence group;
    group.id = 500;
    group.node_set_id = 500;
    group.source_nodes = {10,11};
    declaration.groups.push_back(group);
    rigid.sources = main;
    rigid.plain_rigid_members = {10,11};
    vehicle::rigid_part::Body body;
    body.part_nodes = {11};
    body.extra_nodes = {18};
    rigid.bodies.push_back(body);
    auxiliary.filename = "set-yaris-coarse-v1l.key";
    auxiliary.wall_policy = OriginalWallPolicy::ReplaceWithMeshWall;
    Append(auxiliary.sources, auxiliary.filename.c_str(), "*CONSTRAINED_NODAL_RIGID_BODY", {Card({900,0,900})});
    Append(auxiliary.sources, auxiliary.filename.c_str(), "*ELEMENT_SOLID", {Card({1,1,1,2,3,4,5,6,7,8},8)});
    AuxiliaryGroup extra;
    extra.evidence.id = 900;
    extra.evidence.node_set_id = 900;
    extra.evidence.source_nodes = {40};
    auxiliary.groups.push_back(extra);
    Append(wall_sources,"wall.key","*PART",{"wall",Card({1001,1001,1001})});
    Append(wall_sources,"wall.key","*SECTION_SHELL",{Card({1001,2})});
    Append(wall_sources,"wall.key","*MAT_RIGID",{Card({1001})});
    Append(wall_sources,"wall.key","*NODE",{Card({101},8),Card({102},8),Card({103},8),Card({104},8)});
    Append(wall_sources,"wall.key","*ELEMENT_SHELL",{Card({1001,1001,101,102,103,104},8)});
    Append(wall_sources,"wall.key","*RIGIDWALL_PLANAR_FINITE_ID",{Card({1})});
    const auto& b = wall_sources.back().block;
    auxiliary.original_walls.push_back({b.filename,b.keyword,b.sha256,b.first_line,b.last_line});
    Metadata();
}
void Fixture::Metadata() {
    Document document;
    document.SetObject();
    Value files(rapidjson::kObjectType);
    auto& allocator = document.GetAllocator();
    for (const auto* sources : {&declaration.sources,&auxiliary.sources,&wall_sources}) {
        Value file(rapidjson::kObjectType), blocks(rapidjson::kArrayType), counts(rapidjson::kObjectType);
        std::map<std::string,unsigned> census;
        std::string member;
        for (const auto& source : *sources) {
            Value block(rapidjson::kObjectType);
            const auto text = [&](const char* name, const std::string& value) {
                Value key(name,allocator), field(value.c_str(),value.size(),allocator);
                block.AddMember(key,field,allocator);
            };
            text("file",source.block.filename);
            text("keyword",source.block.keyword);
            text("source_block_sha256",source.block.sha256);
            block.AddMember("first_line",source.block.first_line,allocator);
            block.AddMember("last_line",source.block.last_line,allocator);
            blocks.PushBack(block,allocator);
            ++census[source.block.keyword];
            member += source.block.raw_text;
        }
        for (const auto& [keyword,count] : census) {
            Value key(keyword.c_str(),allocator);
            counts.AddMember(key,count,allocator);
        }
        file.AddMember("blocks",blocks,allocator);
        file.AddMember("keyword_counts",counts,allocator);
        Value hash(Sha256(member).c_str(),allocator);
        file.AddMember("sha256",hash,allocator);
        Value name(sources->front().block.filename.c_str(),allocator);
        files.AddMember(name,file,allocator);
        if (sources == &wall_sources) wall = member;
    }
    document.AddMember("source_files",files,allocator);
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    Require(document.Accept(writer), "Invalid classification fixture JSON");
    canonical.canonical_bytes.assign(buffer.GetString(),buffer.GetSize());
}
ClassificationSourceReceipt Fixture::Check() const {
    ClassificationSourceReceipt receipt;
    classification_detail::ReadSet(declaration,auxiliary,rigid,receipt);
    classification_detail::SourceRoles(canonical,declaration,auxiliary,rigid,receipt,{});
    receipt.wall = classification_detail::Wall(canonical,declaration,wall,{});
    return receipt;
}
}
