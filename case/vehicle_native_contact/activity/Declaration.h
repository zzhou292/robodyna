#pragma once
#include "../SourceAdmission.h"
#include "lib_src/collision/radioss_type25/activity_source/Plan.h"
namespace crash::cases::vehicle_native_contact::activity {
namespace native=tlfea::contact::radioss_type25::activity_source;
enum class Profile { SelectedNativeV6ShellRemoval };
struct Population {
    std::size_t original=0,executed=0,excluded=0,excluded_touching_owner=0;
    std::string executed_ids_sha256,excluded_ids_sha256;
};
struct Coverage {
    Population shells,beams,solids;
    std::size_t physical_nodes=0,wall_shells=0,type13=0,beam18=0,welds=0,joints=0;
    std::array<std::size_t,native::FamilyCount> families{};
    std::string canonical_sha256,scope_sha256,source_member_sha256;
    std::string executed_profile="native_v6_raw8_heph_explicit_cin28";
};
struct Limits {
    std::size_t workspace_bytes=64u<<20,metadata_bytes=1u<<20;
    std::size_t source_host_cap=std::size_t{18}<<30;
};
struct Forecast { std::size_t owned_metadata_bytes=0,construction_workspace_bytes=0; };
// Immutable source declarations only. It grants no runtime activity authority
// and never changes the default or existing contact law. The runtime still
// authenticates this exact joint model against the common physical publisher.
class Declaration {
  public:
    static constexpr Profile profile() noexcept {return Profile::SelectedNativeV6ShellRemoval;}
    static Declaration Prepare(const detail::SourceInputs&,Limits={});
    const native::Controls& self()const noexcept;
    const native::Controls& wall()const noexcept;
    const tl::fea::type45::Model& joints()const noexcept;
    const Coverage& coverage()const noexcept;
    Forecast forecast()const noexcept;
  private:
    struct Data;
    explicit Declaration(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
// Optional source-only report. No owner, transaction, force or GPU history is
// created. Runtime wrappers are supplied by the existing authenticated producer.
std::array<native::Forecast,2> PreflightPlans(const detail::SourceInputs&,
    const Declaration&,const tlfea::contact::radioss_type25::MixedMovingMainSource&,
    const tlfea::contact::radioss_type25::FixedMainSource&,native::Limits={});
output::Document Document(const Declaration&);
}
