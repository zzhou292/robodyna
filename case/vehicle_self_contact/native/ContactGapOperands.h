#pragma once
#include "CorrectedNodalSource.h"
#include "lib_src/collision/RadiossType25GapSource.h"
namespace crash::cases::vehicle_self_contact::native::gap_operands {
namespace n = tlfea::contact::radioss_type25;
namespace values = n::source_gaps;
namespace source = nodal_correction;
enum class Status { Ready, InvalidInput, UnsupportedSource, ResourceLimit, NonfiniteResult, NeedsSourceOrder };
enum class Overrides { FreshPlainPartsAndDefaultGenerators };
enum class Order { CertifiedFiniteGapMaxima };
enum class Family { Quad, Triangle, Beam18, Type13, Type25, Type45 };
struct Report { Status status=Status::InvalidInput; std::string reason; std::uint64_t element=0; };
struct Counts {
    std::size_t nodes=0, shells=0, quads=0, triangles=0, beams=0;
    std::size_t type13=0, type25=0, type45=0, solids_without_direct_gap_term=0;
    std::size_t namespace_only_springs=0;
};
struct Binding {
    Family family=Family::Quad;
    std::uint64_t original_id=0, native_id=0;
    // Source PART when retained in the original contributor. Zero means no
    // retained PID, never a claim that a native generated PART has ID 0.
    std::uint64_t source_part_id=0;
    std::size_t source_index=SIZE_MAX, operand_row=SIZE_MAX;
};
struct SourceProof {
    Overrides overrides=Overrides::FreshPlainPartsAndDefaultGenerators;
    Order order=Order::CertifiedFiniteGapMaxima;
    std::size_t checked_keywords=0, maximum_terms=0, skipped_springs=0;
    bool no_retained_trusses=false;
};
struct Provenance {
    n::UnitScale units;
    std::string source_digest, contributor_digest, property_digest, operand_digest;
};
struct Limits {
    std::size_t host_bytes=std::size_t{8}<<30, shells=524288, beams=1024, springs=16384;
    std::size_t metadata_bytes=1u<<20;
};
struct Forecast {
    std::size_t shared_source=0, shell_rows=0, line_rows=0, spring_rows=0;
    std::size_t bindings=0, source_workspace=0, metadata=0, peak_bytes=0;
};
struct Preparation;
// Complete retained physical source operands only. No invented main/NSV/MSR,
// nodal gap values, scalar interface/search controls, epoch or runtime authority.
class ContactGapOperands {
  public:
    static Forecast Preflight(const source::CorrectedNodalSource&, Limits={});
    static Preparation Prepare(const source::CorrectedNodalSource&, Limits={});
    ContactGapOperands(const ContactGapOperands&) noexcept=default;
    ContactGapOperands(ContactGapOperands&& other) noexcept:data_(other.data_){}
    ContactGapOperands& operator=(const ContactGapOperands&)=delete;
    const source::CorrectedNodalSource& corrected() const noexcept;
    tl::util::ConstView<values::PhysicalShell> shells() const noexcept;
    tl::util::ConstView<values::Line> beams() const noexcept;
    tl::util::ConstView<values::Spring> springs() const noexcept;
    tl::util::ConstView<Binding> bindings() const noexcept;
    const Counts& counts() const noexcept;
    const SourceProof& proof() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Data;
    explicit ContactGapOperands(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
struct Preparation { Report report; std::optional<ContactGapOperands> source; };
}
