#include "TinyFixture.h"
#include "output/BoundedArrayJson.h"
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <iomanip>
#include <sstream>

namespace crash::modelio::tied_shell::test {
using namespace output;
namespace {
Document Object() { Document d; d.SetObject(); return d; }
Document Array() { Document d; d.SetArray(); return d; }
void Push(Document& a, const Document& child) {
    Value v; v.CopyFrom(child, a.GetAllocator()); a.PushBack(v, a.GetAllocator());
}
void Child(Document& d, const char* name, const Document& child) { array_json::Child(d, name, child); }
Document Ids(std::initializer_list<SourceId> ids) {
    auto d = Array();
    for (auto id : ids) d.PushBack(id, d.GetAllocator());
    return d;
}
std::string Card(std::initializer_list<SourceId> ids) {
    std::ostringstream s;
    for (auto id : ids) s << std::setw(10) << id;
    return s.str();
}
struct Builder {
    std::string member;
    Document blocks = Array();
    std::size_t line = 1;
    Document Block(const char* keyword, const std::vector<std::string>& cards) {
        auto d = Object(); auto c = Array();
        const auto first = line;
        std::string raw = std::string(keyword)+"\n"; ++line;
        for (const auto& text : cards) {
            auto row = Object(); Integer(row, "source_line", line++); String(row, "text", text);
            Push(c, row); raw += text+"\n";
        }
        String(d, "file", "yaris-coarse-v1l.key"); String(d, "keyword", keyword);
        Integer(d, "first_line", first); Integer(d, "last_line", line-1);
        String(d, "sha256", Sha256(raw)); Child(d, "cards", c);
        auto original = Object(); original.CopyFrom(d, original.GetAllocator());
        original.RemoveMember("sha256"); String(original, "source_block_sha256", Sha256(raw));
        Push(blocks, original); member += raw;
        return d;
    }
};
template<class T> void AddArray(source::CanonicalData& data, std::size_t index, const char* name,
                              std::initializer_list<T> values, std::size_t width) {
    auto& a = data.arrays[index]; a.name = name;
    const arrays::Layout layout{arrays::detail::Type<T>::value, values.size()/width, width, {}};
    a.bytes = arrays::Encode(layout, values.begin(), values.size());
    a.descriptor = {std::string(name)+".bin", layout, a.bytes.size(), Sha256(a.bytes)};
}
}
std::string Json(const Document& doc) {
    rapidjson::StringBuffer b; rapidjson::Writer<rapidjson::StringBuffer> w(b);
    Require(doc.Accept(w), "Invalid tiny fixture JSON"); return {b.GetString(), b.GetSize()};
}
TinyFixture::TinyFixture(bool set_options, bool search_geometry) {
    Builder b;
    std::size_t first_node_line = 0;
    if (search_geometry) {
        std::vector<std::string> cards;
        const SourceId ids[] = {90,10,80,20,70,30,60,40,50,100,110,120};
        for (unsigned i = 0; i < 12; ++i) {
            std::ostringstream card;
            card << std::setw(8) << ids[i] << std::setw(16) << (i == 0 ? "-0" : i == 1 ? "15.7" : "1")
                 << std::setw(16) << "0" << std::setw(16) << "0";
            cards.push_back(card.str());
        }
        const auto nodes = b.Block("*NODE", cards);
        first_node_line = nodes["first_line"].GetUint64()+1;
    }
    auto contact = b.Block("*CONTACT_TIED_SHELL_EDGE_TO_SURFACE", {Card({2,1,2,2}), "", ""});
    auto slave = b.Block("*SET_PART_LIST_TITLE", {"slave", set_options ? Card({2,1}) : Card({2}), Card({200,201})});
    auto master = b.Block("*SET_PART_LIST_TITLE", {"master", Card({1}), Card({100,101})});
    auto parts = Array(), sections = Array(), materials = Array();
    for (SourceId id : {100,101,200,201}) {
        auto part = b.Block("*PART", {"part", Card({id,id+1000,id+2000})});
        auto section = b.Block(id < 200 ? "*SECTION_SHELL" : id == 200 ? "*SECTION_BEAM" : "*SECTION_SOLID",
                               {Card({id+1000,2}), "  1.000000"});
        auto material = b.Block("*MAT_PIECEWISE_LINEAR_PLASTICITY",
            {search_geometry ? Card({id+2000,1,70000}) : Card({id+2000}), ""});
        Integer(section, "identity", id+1000); Integer(material, "identity", id+2000);
        Push(sections, section); Push(materials, material);
        auto p = Object(), count = Object();
        Integer(p, "source_part_id", id); Integer(p, "source_section_id", id+1000);
        Integer(p, "source_material_id", id+2000);
        Integer(p, "source_line", part["first_line"].GetUint64());
        String(p, "source_part_sha256", detail::Text(part, "sha256"));
        Integer(count, "shells", id < 200); Integer(count, "beams", id == 200); Integer(count, "solids", id == 201);
        Child(p, "counts", count); Push(parts, p);
        canonical.parts.push_back({id,id+2000,id+1000,2,id<200});
    }
    auto groups = Array();
    for (SourceId id : {500,501}) {
        auto group = b.Block("*CONSTRAINED_NODAL_RIGID_BODY", {Card({id,0,id})});
        auto nodes = b.Block("*SET_NODE_LIST", {set_options ? Card({id,1}) : Card({id}), id == 500 ? Card({90,10}) : Card({30,60})});
        auto g = Object(); Integer(g, "identity", id); Integer(g, "node_set_id", id);
        Child(g, "source", group); Child(g, "node_set_source", nodes);
        Child(g, "source_node_ids", id == 500 ? Ids({90,10}) : Ids({30,60})); Push(groups, g);
    }
    b.Block("*CONSTRAINED_JOINT_SPHERICAL_ID", {Card({700}), Card({30,60})});
    auto tied = Object(); tied.AddMember("pairing_qualified", false, tied.GetAllocator());
    Child(tied, "source", contact); Child(tied, "slave_source", slave); Child(tied, "master_source", master);
    Child(tied, "slave_part_ids", Ids({200,201})); Child(tied, "master_part_ids", Ids({100,101}));
    Child(tied, "retained_master_part_ids", Ids({100,101}));
    auto ties = Array(); Push(ties, tied);
    auto connections = Object(); Child(connections, "tied_contacts", ties); Child(connections, "nodal_rigid_groups", groups);
    auto tables = Object(); Child(tables, "section", sections); Child(tables, "material", materials);
    auto declarations = Object(); Child(declarations, "parts", parts); Child(declarations, "tables", tables);
    auto scope = Object(); Child(scope, "connections", connections); Child(scope, "declarations", declarations);
    auto file = Object(); Child(file, "blocks", b.blocks);
    auto files = Object(); Child(files, "yaris-coarse-v1l.key", file);
    auto auxiliary = Object(), auxiliary_blocks = Array(), row = Object();
    String(row, "file", "auxiliary.key"); String(row, "keyword", "*CONSTRAINED_NODAL_RIGID_BODY");
    Integer(row, "first_line", 7); Integer(row, "last_line", 9); String(row, "source_block_sha256", std::string(64,'a'));
    Push(auxiliary_blocks, row); Child(auxiliary, "blocks", auxiliary_blocks); Child(files, "auxiliary.key", auxiliary);
    auto manifest = Object(); Child(manifest, "source_files", files);
    member = std::move(b.member);
    canonical.scope_bytes = Json(scope); canonical.canonical_bytes = Json(manifest);
    canonical.inputs.source_member = {"source.key", Sha256(member), member.size()};
    canonical.selected_parts = {100,101}; canonical.canonical_nodes = 12; canonical.canonical_shells = 2;
    AddArray<SourceId>(canonical, 0, "node_ids", {90,10,80,20,70,30,60,40,50,100,110,120}, 1);
    AddArray<std::int32_t>(canonical, 1, "node_codes", {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, 2);
    AddArray<SourceId>(canonical, 2, "shells_records", {5010,100,90,10,80,20, 5011,101,80,20,70,70}, 6);
    AddArray<std::uint32_t>(canonical, 3, "shells_node_indices", {0,1,2,3, 2,3,4,4}, 4);
    AddArray<SourceId>(canonical, 4, "beams_records", {6000,200,30,60,120,0,0,0,0,0}, 10);
    AddArray<std::uint32_t>(canonical, 5, "beams_node_indices", {5,6}, 2);
    AddArray<SourceId>(canonical, 6, "solids_records", {7000,201,40,50,100,110,120,30,60,90}, 10);
    AddArray<std::uint32_t>(canonical, 7, "solids_node_indices", {7,8,9,10,11,5,6,0}, 8);
    if (search_geometry) {
        canonical.inputs.units = {"t", "mm", "s", 1000, .001, 1};
        AddArray<SourceId>(canonical, 6, "solids_records", {7000,201,40,50,100,110,120,30,30,120}, 10);
        AddArray<std::uint32_t>(canonical, 7, "solids_node_indices", {7,8,9,10,11,5,5,11}, 8);
        AddArray<double>(canonical, 8, "node_positions", {-0.,0,0, 15.7*.001,0,0, .001,0,0,
            .001,0,0, .001,0,0, .001,0,0, .001,0,0, .001,0,0, .001,0,0,
            .001,0,0, .001,0,0, .001,0,0}, 3);
        const auto n = static_cast<std::uint32_t>(first_node_line);
        AddArray<std::uint32_t>(canonical, 9, "node_source_lines", {n,n+1,n+2,n+3,n+4,n+5,n+6,n+7,n+8,n+9,n+10,n+11}, 1);
        AddArray<std::uint16_t>(canonical, 10, "node_blank_masks", {48,48,48,48,48,48,48,48,48,48,48,48}, 1);
    }
}
void TinyFixture::AlterScope(const std::function<void(Document&)>& edit) {
    Document d; d.Parse(canonical.scope_bytes.c_str()); edit(d); canonical.scope_bytes = Json(d);
}
} // namespace crash::modelio::tied_shell::test
