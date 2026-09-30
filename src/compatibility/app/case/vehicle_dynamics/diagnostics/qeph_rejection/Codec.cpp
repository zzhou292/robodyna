#include "Codec.h"
#include "GeometryFields.h"
#include "MaterialFields.h"
#include "MetadataFields.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
namespace {
constexpr std::uint64_t Magic = UINT64_C(0x5145504852454a31);
template<class A, class R> void Record(A& a, R& r) {
    std::uint64_t magic = Magic, version = 1;
    a(magic, version);
    output::Require(magic == Magic && version == 1, "Unsupported QEPH diagnostic word schema");
    codec::Metadata(a, r.metadata);
    a(r.source.available, r.source.parent, r.source.part, r.source.material, r.source.section);
    codec::Reference(a, r.element.reference); a(r.element.nodes);
    codec::Force(a, r.accepted_force, r.element.reference); codec::Interval(a, r.interval);
    a(r.route, r.mapped, r.has_mixed, r.has_failure, r.law);
    codec::Material(a, r); codec::Failure(a, r);
}
}
std::vector<std::uint64_t> EncodeMetadata(const native::RejectedCandidateMetadata& metadata) {
    codec::Writer writer;
    writer(UINT64_C(0x514550484d455431), UINT64_C(1));
    codec::Metadata(writer, metadata);
    return std::move(writer.words);
}
std::vector<std::uint64_t> Encode(const native::RejectedCandidateInput& input) {
    codec::Writer writer; Record(writer, input); return std::move(writer.words);
}
native::RejectedCandidateInput Decode(const std::vector<std::uint64_t>& words) {
    codec::Reader reader(words);
    native::RejectedCandidateInput result;
    Record(reader, result); reader.Finish(); return result;
}
}
