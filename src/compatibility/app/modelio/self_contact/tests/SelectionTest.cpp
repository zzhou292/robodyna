#include "../Internal.h"
#include "modelio/tied_shell/tests/TinyFixture.h"
#include "output/BoundedArrayJson.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <map>
#include <sstream>

namespace crash::modelio::self_contact::test {
namespace {

using output::Document;
using output::Value;

Document Object() {
    Document value;
    value.SetObject();
    return value;
}

Document Array() {
    Document value;
    value.SetArray();
    return value;
}

void Push(Document& array, const Document& child) {
    Value value;
    value.CopyFrom(child, array.GetAllocator());
    array.PushBack(value, array.GetAllocator());
}

std::string Card(std::initializer_list<double> values) {
    std::ostringstream text;
    for (const auto value : values)
        text << std::setw(10) << value;
    return text.str();
}

std::string Ids(std::initializer_list<std::uint64_t> values) {
    std::ostringstream text;
    for (const auto value : values)
        text << std::setw(10) << value;
    return text.str();
}

struct MemberBuilder {
    std::string filename;
    std::string bytes;
    Document blocks = Array();
    std::map<std::string, std::size_t> counts;
    std::size_t line = 1;

    void Add(const char* keyword, const std::vector<std::string>& cards) {
        const auto first = line;
        std::string raw = std::string(keyword) + '\n';
        ++line;
        for (const auto& card : cards) {
            raw += card + '\n';
            ++line;
        }
        Document block = Object();
        output::String(block, "file", filename);
        output::String(block, "keyword", keyword);
        output::Integer(block, "first_line", first);
        output::Integer(block, "last_line", line - 1);
        output::String(block, "source_block_sha256", output::Sha256(raw));
        Push(blocks, block);
        bytes += raw;
        ++counts[keyword];
    }

    Document Description() const {
        Document result = Object();
        output::array_json::Child(result, "blocks", blocks);
        Document census = Object();
        for (const auto& [keyword, count] : counts)
            output::Integer(census, keyword.c_str(), count);
        output::array_json::Child(result, "keyword_counts", census);
        output::String(result, "sha256", output::Sha256(bytes));
        return result;
    }
};

struct Synthetic {
    tied_shell::test::TinyFixture base;
    source::CanonicalData canonical = base.canonical;
    MemberBuilder combine{"combine.key"};
    MemberBuilder auxiliary{"set-yaris-coarse-v1l.key"};

    explicit Synthetic(unsigned set_type = 2, bool duplicate = false,
        bool cycle = false) {
        combine.Add("*CONTACT_AUTOMATIC_SINGLE_SURFACE", {
            Ids({1000002, 0, set_type}),
            Card({.2, .1, .001}), "", Card({1}), "",
            std::string(10, ' ') + Card({1}), "", ""});
        combine.Add("*SET_PART_ADD", {
            Ids({1000002}), Ids({cycle ? 1000002u : 5000002u})});
        auxiliary.Add("*SET_PART_LIST_TITLE", {
            "selected", Ids({5000002}),
            duplicate ? Ids({100, 101, 200, 201, 100})
                      : Ids({100, 101, 200, 201})});

        Document manifest;
        manifest.Parse(canonical.canonical_bytes.c_str());
        output::Require(!manifest.HasParseError(), "Invalid synthetic manifest");
        auto& allocator = manifest.GetAllocator();
        auto add = [&](const char* name, const Document& file) {
            Value copy;
            copy.CopyFrom(file, allocator);
            manifest["source_files"].AddMember(Value(name, allocator),
                copy, allocator);
        };
        add("combine.key", combine.Description());
        add("set-yaris-coarse-v1l.key", auxiliary.Description());
        canonical.canonical_bytes = tied_shell::test::Json(manifest);
    }

    Data Prepare(Limits limits = {}) const {
        detail::Draft draft;
        draft.data.startup_budget_bytes =
            detail::Preflight(canonical, auxiliary.bytes.size(),
                combine.bytes.size(), limits);
        detail::ReadSources(canonical, auxiliary.bytes, combine.bytes,
            draft, limits);
        detail::ResolveCards(draft, limits);
        detail::BuildCensus(canonical, draft, limits);
        draft.data.owned_payload_bytes =
            detail::OwnedPayload(draft.data, limits.host_bytes);
        return std::move(draft.data);
    }
};

}  // namespace

TEST(OriginalSelfContactSelection,
    RecursivePartSetRetainsRawFieldsAndTypedElementDisposition) {
    const Synthetic fixture;
    const auto data = fixture.Prepare();
    EXPECT_EQ(data.sources.size(), 3u);
    EXPECT_EQ(data.source_fields.slave_set_id, 1000002u);
    EXPECT_EQ(data.source_fields.master_set_id, 0u);
    EXPECT_EQ(data.source_fields.slave_set_type, 2u);
    ASSERT_TRUE(data.source_fields.static_friction);
    ASSERT_TRUE(data.source_fields.dynamic_friction);
    ASSERT_TRUE(data.source_fields.decay_coefficient);
    ASSERT_TRUE(data.source_fields.soft);
    ASSERT_TRUE(data.source_fields.ignore_initial_penetration);
    EXPECT_EQ(*data.source_fields.static_friction, .2);
    EXPECT_EQ(*data.source_fields.dynamic_friction, .1);
    EXPECT_EQ(*data.source_fields.decay_coefficient, .001);
    EXPECT_EQ(*data.source_fields.soft, 1);
    EXPECT_EQ(*data.source_fields.ignore_initial_penetration, 1);
    EXPECT_EQ(data.selected_part_ids,
        (std::vector<SourceId>{100, 101, 200, 201}));
    EXPECT_EQ(data.counts.selected_parts, 4u);
    EXPECT_EQ(data.counts.shell_parts, 2u);
    EXPECT_EQ(data.counts.retained_shell_parts, 2u);
    EXPECT_EQ(data.counts.excluded_shell_parts, 0u);
    EXPECT_EQ(data.counts.non_shell_parts, 2u);
    EXPECT_EQ(data.counts.shells, 2u);
    EXPECT_EQ(data.counts.retained_shells, 2u);
    EXPECT_EQ(data.counts.solids, 1u);
    EXPECT_EQ(data.counts.beams, 1u);
    EXPECT_GT(data.startup_budget_bytes, data.owned_payload_bytes);
}

TEST(OriginalSelfContactSelection,
    SourceAuthenticationUnsupportedTypeCyclesAndDuplicatesReject) {
    Synthetic valid;
    detail::Draft draft;
    EXPECT_THROW(detail::ReadSources(valid.canonical,
        valid.auxiliary.bytes, valid.combine.bytes + "x", draft, {}),
        std::exception);
    EXPECT_THROW(Synthetic(3).Prepare(), std::exception);
    EXPECT_THROW(Synthetic(2, true).Prepare(), std::exception);
    EXPECT_THROW(Synthetic(2, false, true).Prepare(), std::exception);
}

TEST(OriginalSelfContactSelection,
    CompleteForecastExactCapAndOneByteShortFailBeforeSourceReads) {
    const Synthetic fixture;
    const auto forecast = detail::Preflight(fixture.canonical,
        fixture.auxiliary.bytes.size(), fixture.combine.bytes.size(), {});
    auto limits = Limits{};
    limits.host_bytes = forecast - 1;
    EXPECT_THROW(detail::Preflight(fixture.canonical,
        fixture.auxiliary.bytes.size(), fixture.combine.bytes.size(), limits),
        std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(detail::Preflight(fixture.canonical,
        fixture.auxiliary.bytes.size(), fixture.combine.bytes.size(), limits),
        forecast);
}

}  // namespace crash::modelio::self_contact::test
