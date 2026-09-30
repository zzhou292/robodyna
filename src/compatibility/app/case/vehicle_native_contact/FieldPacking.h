#pragma once
#include "lib_src/collision/radioss_type25/selection/lifecycle/Types.h"
#include "lib_src/collision/radioss_type25/source_gaps/Types.h"
#include "lib_utils/BoundedStartupArray.h"
#include <vector>
namespace crash::cases::vehicle_native_contact::detail {
namespace native = tlfea::contact::radioss_type25;
namespace lifecycle = native::selection::lifecycle;
enum class NormalPhase { StarterBeforeInitialContact, FixedReady };
struct FieldInputs {
    const native::startup::Snapshot& topology;
    tl::util::ConstView<lifecycle::Node> nodes;
    tl::util::ConstView<double> main_coefficients;
    tl::util::ConstView<native::source_gaps::MainGapFields> main_gaps;
    tl::util::ConstView<std::uint32_t> secondary_nodes;
    tl::util::ConstView<double> secondary_coefficients, secondary_gaps;
};
struct FieldPackingLimits {
    std::size_t nodes = 524288, mains = 1048576, secondaries = 524288;
    std::size_t host_bytes = std::size_t{512} << 20;
};
struct FieldPackingForecast { std::size_t retained_bytes = 0; };
class FieldPacking {
  public:
    // Count-only allocation forecast. It does not validate source descriptors.
    static FieldPackingForecast ForecastStorage(std::size_t nodes, std::size_t mains,
        std::size_t secondaries, FieldPackingLimits = {});
    static FieldPackingForecast Preflight(const FieldInputs&, FieldPackingLimits = {});
    static FieldPacking Starter(const FieldInputs&, FieldPackingLimits = {});
    static FieldPacking FixedReady(const FieldInputs&, const native::startup::FixedMainView&,
                                   FieldPackingLimits = {});
    FieldPacking(FieldPacking&&) noexcept = default;
    FieldPacking& operator=(FieldPacking&&) noexcept = default;
    FieldPacking(const FieldPacking&) = delete;
    FieldPacking& operator=(const FieldPacking&) = delete;
    lifecycle::SourceView view() const noexcept;
    NormalPhase phase() const noexcept { return phase_; }
    const FieldPackingForecast& forecast() const noexcept { return forecast_; }
  private:
    FieldPacking() = default;
    static FieldPacking Pack(const FieldInputs&, const native::startup::NormalView&,
                             NormalPhase, FieldPackingLimits);
    lifecycle::SourceView borrowed_;
    std::vector<lifecycle::Main> mains_;
    std::vector<lifecycle::Secondary> secondary_;
    FieldPackingForecast forecast_;
    NormalPhase phase_ = NormalPhase::StarterBeforeInitialContact;
};
// Numeric field adapter only. The case retains source node/topology/normal
// backing unchanged through every view use. Final removal CSR and history are
// absent here; only GeneralInitialize may install them from PreparedSource.
} // namespace crash::cases::vehicle_native_contact::detail
