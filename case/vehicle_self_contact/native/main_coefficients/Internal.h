#pragma once
#include "../SelectedShellMainSource.h"
#include "../coated/Internal.h"
#include "../coated/TopologyInput.h"
#include "lib_src/collision/RadiossType25MainGeometry.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
#include "lib_src/math/ScalarBits.h"
#include <stdexcept>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace s = coated::s;
namespace a = modelio::assembly;
using output::Require;
struct Failure:std::runtime_error {
    Report report;
    explicit Failure(Report r):std::runtime_error(r.reason), report(std::move(r)){}
};
[[noreturn]] inline void Reject(Status status, const std::string& why, std::size_t primary = SIZE_MAX,
    std::uint64_t contact = 0, std::uint64_t support = 0, std::uint64_t conflicting = 0) {
    throw Failure({status, why, contact, support, conflicting, primary});
}
struct PartValue {
    std::uint64_t pid = 0, sid = 0, mid = 0;
    const a::Material* material = nullptr;
    const a::Section* section = nullptr;
    n::NativeShellMainCoefficientInput coefficient;
    bool native_material_id_preserved = false;
    bool ordinary_part_controls = false;
};
struct ShellValue { std::size_t part = SIZE_MAX; };
struct FaceKey {
    std::uint32_t nodes[4]{};
    std::uint32_t physical = 0;
    unsigned arity = 0;
};
struct Packed {
    std::vector<PartValue> parts;
    std::vector<ShellValue> shells;
    std::vector<FaceKey> keys;
};
Forecast Budget(const c::CorrectedNodalSource&, const modelio::self_contact::OriginalSelection&, Limits);
void CheckContext(const c::CorrectedNodalSource&, const modelio::self_contact::OriginalSelection&);
bool GroupingContext(const modelio::self_contact::OriginalSelection&, const std::string& combine_member);
bool GroupingKeywords(const output::Value& source_files);
bool NoOptionalAmsCard(const std::vector<modelio::tied_shell::SourceEvidence>&);
bool OrdinaryPartControls(const output::Value&);
Packed PackShells(const c::CorrectedNodalSource&, const coated::Inputs&, Limits);
bool KeyLess(const FaceKey&, const FaceKey&) noexcept;
bool SameKey(const FaceKey&, const FaceKey&) noexcept;
FaceKey Key(const coated::Shell&, std::size_t);
bool GroupPrefixEqual(const PartValue&, const PartValue&) noexcept;
struct SupportSelection {
    std::vector<std::size_t> winners;
    std::size_t owner = SIZE_MAX;
    OwnerProof proof = OwnerProof::UnresolvedEqualValues;
};
SupportSelection SelectSupport(const coated::Inputs&, const Packed&, const s::Main&,
    std::size_t physical, bool grouping_context);
void AccumulateSupport(const Packed&, std::size_t, SupportSelection&, double& thickness, double& young);
SupportSelection ResolveSupportTies(const coated::Inputs&, const Packed&, const s::Main&,
    SupportSelection, bool triangle, bool grouping_context);
void CheckIdentity(const s::Main&, const n::NativeExteriorMainGeometryResult&,
    std::size_t primary, std::uint64_t eid);
std::string Digest(const Provenance&, const Certificate&, const std::vector<PrimaryBinding>&,
    const std::vector<CandidateOwner>&, const std::vector<double>&, std::size_t);
} // namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail
