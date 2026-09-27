#pragma once
#include "modelio/solid_control/EffectiveSource.h"
#include "ContactNodalSeed.h"
#include "lib_src/collision/RadiossType25NodalCorrection.h"
#include <optional>

namespace crash::cases::vehicle_self_contact::native::nodal_correction {
namespace n = tlfea::contact::radioss_type25;
namespace seed = nodal_seed;
enum class Status {
    Ready, InvalidInput, UnsupportedSource, ResourceLimit, NonfiniteResult,
    NeedsNativePropertyMapping, NeedsNativeStorageOrder
};
enum class OrderPolicy { CertifiedEqualNodeFactors };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason, source_file;
    std::size_t source_line = 0;
    std::uint64_t first_element = 0, conflicting_element = 0, source_node = 0;
};
using PartControl = modelio::solid_control::PartControl;
using InterfaceDisposition = modelio::solid_control::InterfaceDisposition;
using InterfaceCensus = modelio::solid_control::InterfaceCensus;
struct Provenance {
    OrderPolicy order = OrderPolicy::CertifiedEqualNodeFactors;
    std::string source_digest, pre_correction_digest, property_digest, material_digest, certificate_digest;
    n::source_nodal::correction::OrderCertificate certificate;
    InterfaceCensus interfaces;
};
struct Limits {
    std::size_t host_bytes = std::size_t{8} << 30;
    std::size_t nodes = 524288, solids = 16384, parts = 4096, source_blocks = 8192;
    std::size_t metadata_bytes = 1u << 20;
};
struct Forecast {
    std::size_t pre_correction_reservation = 0, import_context_reservation = 0;
    std::size_t source_workspace = 0, correction_inputs = 0;
    std::size_t certificate_scratch = 0, correction_scratch = 0, output_bytes = 0;
    std::size_t peak_bytes = 0;
};
enum class MaterialSlotStatus { Ready, UnknownPart, NotRetainedSolidPart, UnsupportedMaterial, InvalidMaterial };
struct RetainedMaterialSlots {
    std::uint64_t part_id = 0, section_id = 0, material_id = 0;
    n::UnitScale units;
    double pm32 = 0, pm100 = 0, pm107 = 0;
};
struct MaterialSlotQuery {
    MaterialSlotStatus status = MaterialSlotStatus::UnknownPart;
    std::optional<RetainedMaterialSlots> values;
};
struct Preparation;

// Global startup nodal contact K after the native distortion-control correction.
// This source product is separate from interface secondary scaling, main K,
// gaps, contact/removal readiness, physical ownership and runtime admission.
// Construction authenticates the whole source/import context internally; no
// caller-provided control flags, material slots or certificate authorize output.
class CorrectedNodalSource {
  public:
    CorrectedNodalSource(const CorrectedNodalSource&) noexcept = default;
    CorrectedNodalSource(CorrectedNodalSource&& other) noexcept : data_(other.data_) {}
    CorrectedNodalSource& operator=(const CorrectedNodalSource&) = delete;
    static Forecast Preflight(const seed::PreCorrectionNodalSource&,
        const modelio::native_spring_ids::ImportMembers&, Limits = {});
    static Preparation Prepare(const seed::PreCorrectionNodalSource&,
        const modelio::native_spring_ids::ImportMembers&, Limits = {});
    const seed::PreCorrectionNodalSource& pre_correction() const noexcept;
    const Forecast& forecast() const noexcept;
    const Provenance& provenance() const noexcept;
    const std::vector<PartControl>& part_controls() const noexcept;
    const modelio::solid_control::EffectiveSource& solid_control_source() const noexcept;
    tl::util::ConstView<double> coefficients() const noexcept;
    // Native pressure-valued slots AFTER HM_READ_MAT/UPDMAT, in the retained
    // declared units. Reads the same qualified material helper as correction;
    // never substitutes global corrected K for main K. Only retained solid
    // parts are available. Effective property control remains part_controls().
    MaterialSlotQuery material_slots(std::uint64_t source_part_id) const noexcept;
  private:
    struct Data;
    explicit CorrectedNodalSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
struct Preparation {
    Report report;
    // Disengaged for every rejected/unsupported case, including conflicting
    // shared factors. Diagnostic source IDs do not grant coefficient authority.
    std::optional<CorrectedNodalSource> source;
};
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction
