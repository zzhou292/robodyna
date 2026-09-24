#include "CandidateFailureValues.h"
#include "FailureBaselineScope.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::failure_detail {
bool ConsumeBaselineObserved(output::Document& manifest) {
    using namespace output;
    Require(manifest.IsObject() && manifest.HasMember("schema"),
            "Failure fixture metadata schema is absent");
    const auto schema = array_json::Text(manifest["schema"]);
    Require(schema == LegacyFailureManifestSchema || schema == FailureManifestSchema,
            "Unsupported failure fixture metadata version");
    unsigned occurrences = 0;
    for (auto field = manifest.MemberBegin(); field != manifest.MemberEnd(); ++field)
        occurrences += std::string_view(field->name.GetString(), field->name.GetStringLength()) ==
                       "baseline_observed";
    if (schema == LegacyFailureManifestSchema) {
        Require(occurrences == 0, "Legacy failure fixture has unexpected baseline metadata");
        return false;
    }
    Require(occurrences == 1 && manifest["baseline_observed"].IsBool(),
            "Failure baseline_observed must occur once and be boolean");
    const bool observed = manifest["baseline_observed"].GetBool();
    manifest.RemoveMember("baseline_observed");
    return observed;
}
sct::NonlinearSeparationResult CapturedNonlinearBaseline(
    const sct::CandidateFailureCapture& input) noexcept {
    return input.has_nonlinear_baseline ? input.nonlinear_baseline
                                        : sct::NonlinearSeparationResult{};
}
prepared_replay::PairResult Evaluate(const nonlinear_fixture::Pair& pair, double duration,
    std::size_t work, unsigned depth, const sct::AcceptedEventCertificate* owners,
    std::size_t count, bool observed_nonlinear_baseline) {
    prepared_replay::PairResult r;
    r.family = "failure";
    r.file = "pair.bin";
    r.owners = count;
    r.affine = true;
    r.baseline_status = pair.baseline_status;
    r.baseline_work = pair.baseline_work;
    r.baseline_depth = pair.baseline_depth;
    r.observed_failure_baseline = observed_nonlinear_baseline;
    for (unsigned side = 0; side < 2; ++side) {
        r.facets[side] = pair.prepared[side].key;
        for (const auto& vertex : pair.quadratic[side].q)
            for (const auto& q : vertex)
                r.affine = r.affine && q.lower == 0 && q.upper == 0;
    }
    const auto exclusions = prepared_replay::SameRigidExclusions(pair);
    r.exclusions = exclusions.size();
    r.ledger = sct::CertifyQuadraticFacetCoverage(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1],
        duration, owners, count, work, depth);
    r.policy = sct::CertifyQuadraticFacetPolicyCoverage(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1],
        duration, owners, count, exclusions.data(), exclusions.size(), work, depth);
    return r;
}
bool Equivalent(const prepared_replay::PairResult& a, const prepared_replay::PairResult& b) {
    auto first = prepared_replay::PairDocument(a);
    auto second = prepared_replay::PairDocument(b);
    for (const char* field : {"ledger", "policy"}) {
        first[field].RemoveMember("frozen_certificate_ordinal");
        second[field].RemoveMember("frozen_certificate_ordinal");
        if (first[field] != second[field])
            return false;
    }
    return true;
}
std::uint64_t ProfileHash(std::size_t work, unsigned depth, std::size_t nw, unsigned nd) {
    std::uint64_t h = 1469598103934665603ull;
    for (const std::uint64_t value : {std::uint64_t(work), std::uint64_t(depth), std::uint64_t(nw), std::uint64_t(nd)})
        nonlinear_fixture::HashUnsigned(value, &h);
    return h;
}
std::uint64_t DtHash(double duration, double kick, std::uint64_t trajectory) {
    std::uint64_t h = 1469598103934665603ull;
    for (const auto value : {output::Bits(duration), output::Bits(kick), trajectory})
        nonlinear_fixture::HashUnsigned(value, &h);
    return h;
}
output::Document Geometry(const nonlinear_fixture::Pair& pair) {
    using namespace output;
    Document result;
    result.SetObject();
    result.AddMember("facets", Value(rapidjson::kArrayType), result.GetAllocator());
    for (unsigned side = 0; side < 2; ++side) {
        Document d;
        d.SetObject();
        Integer(d, "inventory_facet", pair.facet[side]);
        Number(d, "half_thickness_m", pair.half_thickness[side]);
        Integer(d, "half_thickness_bits", Bits(pair.half_thickness[side]));
        d.AddMember("vertices", Value(rapidjson::kArrayType), d.GetAllocator());
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            Document v;
            v.SetObject();
            const auto& key = pair.accepted[side].vertex_keys[vertex];
            Integer(v, "source_instance_id", key.source_instance_id);
            Integer(v, "kind", unsigned(key.kind));
            Integer(v, "first", key.first);
            Integer(v, "second", key.second);
            Integer(v, "numerator", key.numerator);
            Integer(v, "denominator", key.denominator);
            Integer(v, "level", key.level);
            Integer(v, "grid_i", key.grid_i);
            Integer(v, "grid_j", key.grid_j);
            const auto a = pair.accepted[side].vertices[vertex];
            const auto p = pair.prepared[side].vertices[vertex];
            const double xyz[]{a.x, a.y, a.z, p.x, p.y, p.z};
            FiniteArray(v, "accepted_then_prepared_xyz", xyz, 6);
            Value bits(rapidjson::kArrayType);
            for (double value : xyz)
                bits.PushBack(Bits(value), v.GetAllocator());
            v.AddMember("coordinate_bits", bits, v.GetAllocator());
            const auto& q = pair.quadratic[side].q[vertex];
            const double bounds[]{q[0].lower, q[0].upper, q[1].lower, q[1].upper, q[2].lower, q[2].upper};
            FiniteArray(v, "quadratic_lower_upper_xyz", bounds, 6);
            Value qb(rapidjson::kArrayType);
            for (double value : bounds)
                qb.PushBack(Bits(value), v.GetAllocator());
            v.AddMember("quadratic_bits", qb, v.GetAllocator());
            Value child;
            child.CopyFrom(v, d.GetAllocator());
            d["vertices"].PushBack(child, d.GetAllocator());
        }
        Value child;
        child.CopyFrom(d, result.GetAllocator());
        result["facets"].PushBack(child, result.GetAllocator());
    }
    Integer(result, "accepted_mask", pair.accepted_mask);
    Integer(result, "prepared_mask", pair.prepared_mask);
    return result;
}
std::string JsonBytes(const output::Document& document) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(document.Accept(writer), "Cannot encode failure fixture JSON");
    output::Require(buffer.GetSize() <= FailureFixtureManifestCap, "Failure fixture JSON exceeds cap");
    return {buffer.GetString(), buffer.GetSize()};
}
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::failure_detail
