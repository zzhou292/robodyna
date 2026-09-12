#pragma once
#include "modelio/tied_shell/TiedShellDeclaration.h"
#include "lib_src/elements/beam18/ForceTypes.h"
namespace crash::modelio::beam18 {
namespace source=output::full_shell::source;
namespace native=tl::fea::beam18;
enum class Policy { OriginalCircularFourPointLaw44V1 };
struct Limits {
    std::size_t host_bytes=512u<<20,member_bytes=64u<<20,metadata_bytes=1u<<20;
    std::size_t parents=1024,nodes=512,canonical_nodes=524288,source_beams=8192,blocks=16384;
};
struct Forecast {std::size_t total_bytes=0;};
struct Part {
    std::uint64_t id=0,section_id=0,material_id=0;
    std::array<std::size_t,3> sources{};
    std::size_t curve_source=SIZE_MAX;
    double radius_mm=0,density_working=0,young_working=0;
    // Original declared Ismstr4 resolves to stored ISMSTR0, df0 to .01.
    unsigned converter_property=18,declared_small_strain=4,stored_small_strain=0;
    native::Material material;
};
struct Node {
    std::uint64_t id=0;
    std::uint32_t canonical_index=0,source_line=0;
    std::uint16_t blank_mask=0;
    native::Vec3 position_working{},position_m{};
    std::string raw_card;
};
struct Row {
    std::uint64_t element_id=0,part_id=0;
    std::uint32_t canonical_row=0,source_line=0,part_index=0;
    std::uint16_t blank_mask=0;
    std::array<std::uint64_t,10> raw_record{};
    std::array<std::uint32_t,3> nodes{}; // Third entry is evidence only.
    std::string raw_card;
    native::Reference reference;
};
struct Data {
    Policy policy=Policy::OriginalCircularFourPointLaw44V1;
    std::vector<tied_shell::SourceEvidence> sources;
    std::vector<Part> parts;
    std::vector<Node> nodes; // Sorted original NID, includes orientation evidence.
    std::vector<Row> rows; // Original canonical element order.
    std::vector<std::uint32_t> canonical_endpoints; // Sorted; N3 contributes no role.
    std::vector<double> plastic_strain,yield_stress_pa;
    std::size_t original_beams=0,outside_beams=0,owned_payload_bytes=0;
};
class Source {
  public:
    static Forecast Preflight(const source::CanonicalSource&,Policy,Limits={});
    static Source Prepare(const source::CanonicalSource&,const std::string& member,Policy,Limits={});
    Source(const Source&) noexcept=default;
    Source(Source&& other) noexcept:storage_(other.storage_){}
    Source& operator=(const Source&)=delete;
    const source::CanonicalSource& canonical() const noexcept;
    const Data& data() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Storage;
    explicit Source(std::shared_ptr<const Storage> value):storage_(std::move(value)){}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::beam18
