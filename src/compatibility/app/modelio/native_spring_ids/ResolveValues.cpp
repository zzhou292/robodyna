#include "Internal.h"
#include <algorithm>
#include <climits>
#include <numeric>
#include <set>
namespace crash::modelio::native_spring_ids {
namespace detail {
namespace {
auto Key(const SourceRow& row) { return std::make_pair(unsigned(row.kind), row.original_id); }
void Validate(const SourceRow& row) {
    if (unsigned(row.kind) > unsigned(SourceKind::RegularJoint) || !row.original_id || row.original_id > INT_MAX ||
        !row.endpoints[0] || !row.endpoints[1] || row.endpoints[0] > INT_MAX || row.endpoints[1] > INT_MAX ||
        row.location.file.size() > 128)
        Reject(Readiness::InvalidSource, "Malformed source identity in SPRING order", row.location.file, row.location.line, row.original_id);
}
void Text(std::string& bytes, const std::string& value) {
    const std::uint64_t size = value.size();
    bytes += output::arrays::Encode<std::uint64_t>({output::arrays::Scalar::UInt64,1,1,{}}, &size, 1);
    bytes += value;
}
}
std::string DigestRows(const std::vector<Row>& rows, const std::string& binding) {
    std::string bytes = "robo_dyna.native_spring_ids.v1"; Text(bytes, binding);
    for (const auto& row : rows) {
        const std::uint64_t words[]{unsigned(row.kind), row.original_id, row.native_id, row.endpoints[0], row.endpoints[1],
            row.source_index, row.canonical_index, row.location.line, std::uint64_t(row.physical_participant)};
        bytes += output::arrays::Encode<std::uint64_t>({output::arrays::Scalar::UInt64,1,9,{}}, words, 9);
        Text(bytes, row.location.file);
    }
    return output::Sha256(bytes);
}
Resolution ResolveRows(const ContextData& context, std::vector<SourceRow> retained, Limits limits) {
    CheckLimits(limits);
    Resolution result;
    if (context.diagnostic.status != Readiness::Ready) { result.diagnostic = context.diagnostic; return result; }
    if (context.profile != Profile::DirectKeywordR14FreshRadiossPoSortById)
        Reject(Readiness::UnsupportedProfile, "Unresolved native SPRING import policy");
    output::arrays::CheckHash(context.source_digest);
    const auto total = context.precursors.size()+context.welds.size()+context.joints.size();
    if (total > limits.rows || retained.size() > limits.rows ||
        total > limits.resolve_bytes/(4*(sizeof(Row)+256)))
        Reject(Readiness::ResourceLimit, "SPRING mapping workspace exceeds cap");
    std::sort(retained.begin(), retained.end(), [](const auto& a, const auto& b) { return Key(a) < Key(b); });
    for (std::size_t i = 0; i < retained.size(); ++i) {
        Validate(retained[i]);
        if (retained[i].kind == SourceKind::DiscreteNamespaceOnly || retained[i].source_index == SIZE_MAX ||
            (i && Key(retained[i-1]) == Key(retained[i])))
            Reject(Readiness::IdentityMismatch, "Invalid or duplicate retained source binding");
    }
    std::vector<unsigned char> used(retained.size(), 0);
    std::set<std::uint64_t> existing;
    std::set<std::pair<unsigned,std::uint64_t>> source_ids;
    result.rows.reserve(total);
    const auto append = [&](const SourceRow& source, std::uint64_t native) {
        Validate(source);
        if (!source_ids.insert(Key(source)).second)
            Reject(Readiness::InvalidSource, "Duplicate original SPRING source identity", source.location.file, source.location.line, source.original_id);
        Row row; static_cast<SourceRow&>(row) = source; row.native_id = native;
        if (source.kind != SourceKind::DiscreteNamespaceOnly) {
            const auto found = std::lower_bound(retained.begin(), retained.end(), Key(source),
                [](const auto& a, const auto& key) { return Key(a) < key; });
            if (found == retained.end() || Key(*found) != Key(source) || found->endpoints != source.endpoints)
                Reject(Readiness::IdentityMismatch, "Retained ID/endpoints differ from complete import source", source.location.file, source.location.line, source.original_id);
            row.source_index = found->source_index; row.physical_participant = found->physical_participant;
            used[std::size_t(found-retained.begin())] = 1;
        } else {
            row.source_index = SIZE_MAX; row.physical_participant = false;
        }
        result.rows.push_back(std::move(row));
    };
    std::uint64_t maximum = 0;
    for (const auto& source : context.precursors) {
        if (source.kind != SourceKind::Type13 && source.kind != SourceKind::DiscreteNamespaceOnly)
            Reject(Readiness::InvalidSource, "Generated record appears in precursor namespace");
        if (!existing.insert(source.original_id).second)
            Reject(Readiness::InvalidSource, "Duplicate explicit SPRING destination ID");
        maximum = std::max(maximum, source.original_id);
        append(source, source.original_id);
    }
    result.existing_maximum = maximum;
    const auto generated = context.welds.size()+context.joints.size();
    if (maximum > INT_MAX || generated > std::size_t(INT_MAX-maximum))
        Reject(Readiness::Overflow, "Generated SPRING range exceeds positive native integer IDs");
    const auto generate = [&](const std::vector<SourceRow>& source, SourceKind kind) {
        std::vector<std::uint32_t> order(source.size()); std::iota(order.begin(), order.end(), 0u);
        std::sort(order.begin(), order.end(), [&](auto a, auto b) { return source[a].original_id < source[b].original_id; });
        for (const auto index : order) {
            if (source[index].kind != kind) Reject(Readiness::InvalidSource, "Wrong generated source family");
            append(source[index], ++maximum);
        }
    };
    // One proved default weld group, then actual regular joint selection order.
    generate(context.welds, SourceKind::DefaultSpotweld);
    generate(context.joints, SourceKind::RegularJoint);
    if (std::find(used.begin(), used.end(), 0) != used.end())
        Reject(Readiness::IdentityMismatch, "Retained contributor is absent from complete import namespace");
    std::sort(result.rows.begin(), result.rows.end(), [](const auto& a, const auto& b) { return a.native_id < b.native_id; });
    result.source_order.resize(result.rows.size()); std::iota(result.source_order.begin(), result.source_order.end(), 0u);
    std::sort(result.source_order.begin(), result.source_order.end(), [&](auto a, auto b) { return Key(result.rows[a]) < Key(result.rows[b]); });
    for (std::size_t i = 0; i < result.rows.size(); ++i) {
        const auto& row = result.rows[i];
        if (i && result.rows[i-1].native_id == row.native_id) Reject(Readiness::IdentityMismatch, "Duplicate produced SPRING ID");
        switch (row.kind) {
        case SourceKind::Type13: ++result.counts.type13; break;
        case SourceKind::DiscreteNamespaceOnly: ++result.counts.discrete_namespace_only; break;
        case SourceKind::DefaultSpotweld: ++result.counts.default_welds; break;
        case SourceKind::RegularJoint: ++result.counts.regular_joints; break;
        }
        if (row.physical_participant) { result.physical_order.push_back(std::uint32_t(i)); ++result.counts.physical; }
    }
    result.counts.non_spring_beams = context.non_spring_beams;
    result.final_maximum = maximum; result.source_digest = context.source_digest;
    result.mapping_digest = DigestRows(result.rows, result.source_digest);
    result.diagnostic = {Readiness::Ready, "Complete source-derived native SPRING identities", {}, 0, 0};
    return result;
}
}
const Row* Resolution::Find(SourceKind kind, std::uint64_t id) const noexcept {
    if (diagnostic.status != Readiness::Ready) return nullptr;
    const auto key = std::make_pair(unsigned(kind), id);
    const auto found = std::lower_bound(source_order.begin(), source_order.end(), key,
        [&](auto index, const auto& value) {
            if (index >= rows.size()) return false;
            return std::make_pair(unsigned(rows[index].kind), rows[index].original_id) < value;
        });
    if (found == source_order.end() || *found >= rows.size()) return nullptr;
    const auto& row = rows[*found];
    return row.kind == kind && row.original_id == id ? &row : nullptr;
}
} // namespace crash::modelio::native_spring_ids
