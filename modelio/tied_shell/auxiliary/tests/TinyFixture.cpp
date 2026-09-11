#include "TinyFixture.h"
#include "output/BoundedArrayJson.h"
#include <iomanip>
#include <sstream>

namespace crash::modelio::tied_shell::auxiliary_test {
using namespace output;
namespace {
Document Object() { Document d; d.SetObject(); return d; }
Document Array() { Document d; d.SetArray(); return d; }
std::string Card(std::initializer_list<std::string> values) {
    std::ostringstream out;
    for (const auto& value : values) out << std::setw(10) << value;
    return out.str();
}
void Counts(Value& file, Document::AllocatorType& alloc) {
    std::map<std::string, unsigned> counts;
    for (const auto& row : file["blocks"].GetArray()) ++counts[detail::Text(row, "keyword")];
    Value value(rapidjson::kObjectType);
    for (const auto& [name, count] : counts) {
        Value key; key.SetString(name.c_str(), name.size(), alloc);
        value.AddMember(key, count, alloc);
    }
    file.AddMember("keyword_counts", value, alloc);
}
}
Tiny::Tiny() {
    Document canonical;
    canonical.Parse(base.canonical.canonical_bytes.c_str());
    auto& files = canonical["source_files"];
    files.RemoveMember("auxiliary.key");
    Counts(files["yaris-coarse-v1l.key"], canonical.GetAllocator());
    auto blocks = Array();
    std::size_t line = 1;
    const auto block = [&](const char* keyword, std::initializer_list<std::string> cards) {
        auto value = Object();
        const auto first = line;
        std::string raw = std::string(keyword)+"\n";
        ++line;
        for (const auto& card : cards) { raw += card+"\n"; ++line; }
        String(value, "file", auxiliary_detail::MemberName);
        String(value, "keyword", keyword);
        Integer(value, "first_line", first);
        Integer(value, "last_line", line-1);
        String(value, "source_block_sha256", Sha256(raw));
        Value row; row.CopyFrom(value, blocks.GetAllocator()); blocks.PushBack(row, blocks.GetAllocator());
        member += raw;
    };
    block("*CONSTRAINED_NODAL_RIGID_BODY", {Card({"900", "", "910"})});
    block("*SET_NODE_LIST", {Card({"910"}), Card({"10", "901", "30"})});
    block("*CONSTRAINED_NODAL_RIGID_BODY", {Card({"902", "0", "912"})});
    block("*SET_NODE_LIST", {Card({"912"}), Card({"20", "903"})});
    block("*NODE", {"     901             1.0             2.0             3.0",
                    "     903             4.0             5.0             6.0       0       0"});
    auto file = Object();
    String(file, "sha256", Sha256(member));
    array_json::Child(file, "blocks", blocks);
    Counts(file, file.GetAllocator());
    Value node; node.CopyFrom(file, canonical.GetAllocator());
    files.AddMember(Value(auxiliary_detail::MemberName, canonical.GetAllocator()), node, canonical.GetAllocator());
    auto wall = Object(), wall_blocks = Array(), wall_block = Object();
    const std::string wall_raw = "*RIGIDWALL_PLANAR\nwall declaration\n";
    String(wall_block, "file", "wall.key");
    String(wall_block, "keyword", "*RIGIDWALL_PLANAR");
    String(wall_block, "source_block_sha256", Sha256(wall_raw));
    Integer(wall_block, "first_line", 1); Integer(wall_block, "last_line", 2);
    Value row; row.CopyFrom(wall_block, wall_blocks.GetAllocator()); wall_blocks.PushBack(row, wall_blocks.GetAllocator());
    array_json::Child(wall, "blocks", wall_blocks); String(wall, "sha256", Sha256(wall_raw));
    Counts(wall, wall.GetAllocator());
    Value wall_value; wall_value.CopyFrom(wall, canonical.GetAllocator());
    files.AddMember("wall.key", wall_value, canonical.GetAllocator());
    base.canonical.canonical_bytes = test::Json(canonical);
}
AuxiliaryData Tiny::Prepare(OriginalWallPolicy policy, AuxiliaryLimits limits) const {
    const auto declaration = base.Prepare();
    return auxiliary_detail::Build(base.canonical, declaration, member, policy, limits);
}
void Tiny::AlterCanonical(const std::function<void(Document&)>& edit) {
    Document d; d.Parse(base.canonical.canonical_bytes.c_str()); edit(d);
    base.canonical.canonical_bytes = test::Json(d);
}
void Tiny::AlterMember(const std::function<void(std::string&)>& edit, bool rehash_blocks) {
    edit(member);
    AlterCanonical([&](auto& d) {
        auto& file = d["source_files"][auxiliary_detail::MemberName];
        const auto hash = Sha256(member); file["sha256"].SetString(hash.c_str(), d.GetAllocator());
        if (!rehash_blocks) return;
        std::vector<std::size_t> offsets{0};
        for (std::size_t i = 0; i < member.size(); ++i) if (member[i]=='\n') offsets.push_back(i+1);
        for (auto& row : file["blocks"].GetArray()) {
            const auto first = detail::Unsigned(row, "first_line"), last = detail::Unsigned(row, "last_line");
            const auto block_hash = Sha256(member.substr(offsets.at(first-1), offsets.at(last)-offsets.at(first-1)));
            row["source_block_sha256"].SetString(block_hash.c_str(), d.GetAllocator());
        }
    });
}
} // namespace crash::modelio::tied_shell::auxiliary_test
