#include "Options.h"
#include <charconv>
#include <cmath>
#include <set>
#include <stdexcept>
namespace crash::cases::vehicle_run::cli {
namespace {
double Real(const std::string& text) {
    std::size_t used=0;
    const double value=std::stod(text,&used);
    if(used!=text.size() || !std::isfinite(value)) throw std::invalid_argument("Invalid finite CLI number");
    return value;
}
std::uint64_t Integer(const std::string& text) {
    std::uint64_t result=0;
    const auto parsed=std::from_chars(text.data(),text.data()+text.size(),result);
    if(parsed.ec!=std::errc{} || parsed.ptr!=text.data()+text.size()) throw std::invalid_argument("Invalid unsigned CLI integer");
    return result;
}
}
Options Parse(int argc,const char* const* argv) {
    if(argc<1 || argc>64) throw std::invalid_argument("CLI argument count exceeds its bounded profile");
    Options result;
    std::set<std::string> supplied;
    for(int i=1;i<argc;++i) {
        if(std::char_traits<char>::length(argv[i])>4096) throw std::invalid_argument("CLI option exceeds path/text cap");
        const std::string name=argv[i];
        if(!supplied.insert(name).second) throw std::invalid_argument("Duplicate CLI option");
        if(name=="--forecast-only") {result.forecast_only=true;continue;}
        if(name=="--conditional-full-limits") {result.config.resources=ResourceProfile::ConditionalExpandedFull;continue;}
        if(i+1==argc) throw std::invalid_argument("Missing CLI option value");
        if(std::char_traits<char>::length(argv[i+1])>4096) throw std::invalid_argument("CLI value exceeds path/text cap");
        const std::string value=argv[++i];
        if(name=="--canonical") result.source.canonical=value;
        else if(name=="--scope") result.source.scope=value;
        else if(name=="--member") result.source.member=value;
        else if(name=="--declarations") result.source.declarations=value;
        else if(name=="--glass-resolution") result.source.glass_resolution=value;
        else if(name=="--type13") result.source.type13=value;
        else if(name=="--aux-member") result.source.auxiliary_member=value;
        else if(name=="--original-wall-member") result.source.original_wall_member=value;
        else if(name=="--wall-manifest") result.source.wall_manifest=value;
        else if(name=="--self-contact-member") result.source.self_contact_combine_member=value;
        else if(name=="--output") result.output=value;
        else if(name=="--self-contact-failure-output") result.failure_output=value;
        else if(name=="--stop-file") result.stop_file=value;
        else if(name=="--physical-profile") {
            if(value==PhysicalProfileName(PhysicalProfile::RetainedShellAssembliesV1))
                result.config.physical_profile=PhysicalProfile::RetainedShellAssembliesV1;
            else if(value==PhysicalProfileName(PhysicalProfile::ExtendedSolidsV4))
                result.config.physical_profile=PhysicalProfile::ExtendedSolidsV4;
            else if(value==PhysicalProfileName(PhysicalProfile::VehicleSupportsV5))
                result.config.physical_profile=PhysicalProfile::VehicleSupportsV5;
            else throw std::invalid_argument("Unknown Yaris physical profile");
        }
        else if(name=="--contact-profile") {
            if(value==ContactProfileName(ContactProfile::WallOnly))
                result.config.contact_profile=ContactProfile::WallOnly;
            else if(value==ContactProfileName(ContactProfile::WallSelfContactV1))
                result.config.contact_profile=ContactProfile::WallSelfContactV1;
            else throw std::invalid_argument("Unknown contact profile");
        }
        else if(name=="--duration-ms") result.config.duration_s=Real(value)/1000;
        else if(name=="--fixed-dt-s") result.config.fixed_dt_s=Real(value);
        else if(name=="--gap-m") result.gap_m=Real(value);
        else if(name=="--wall-stiffness-n-m3") result.wall_stiffness_n_m3=Real(value);
        else if(name=="--penetration-limit-m") result.penetration_limit_m=Real(value);
        else if(name=="--samples") result.config.samples=Integer(value);
        else if(name=="--diagnostic-intervals") result.diagnostic_intervals=Integer(value);
        else if(name=="--maximum-elapsed-s") result.maximum_elapsed_s=Real(value);
        else if(name=="--run-id") result.run_id=Integer(value);
        else throw std::invalid_argument("Unknown CLI option: "+name);
    }
    Plan(result.config);
    if(result.config.contact_profile==ContactProfile::WallSelfContactV1 &&
        result.source.self_contact_combine_member.empty())
        throw std::invalid_argument("Wall+self contact requires an explicit pinned --self-contact-member");
    if(result.config.contact_profile==ContactProfile::WallOnly &&
        !result.source.self_contact_combine_member.empty())
        throw std::invalid_argument("A self-contact member cannot be silently ignored by the wall-only profile");
    if(!result.failure_output.empty() &&
        (result.config.contact_profile!=ContactProfile::WallSelfContactV1 || result.output.empty()))
        throw std::invalid_argument("Failure diagnostics require wall+self contact and an explicit --output");
    if(supplied.count("--self-contact-failure-output") && result.failure_output.empty())
        throw std::invalid_argument("Failure diagnostic destination must be nonempty");
    for(const auto value:{result.wall_stiffness_n_m3,result.penetration_limit_m})
        if(value && *value<=0) throw std::invalid_argument("Wall stiffness and penetration limit must be positive");
    if(!result.run_id || result.gap_m<=0 || result.maximum_elapsed_s<0 || (!result.forecast_only && result.output.empty()))
        throw std::invalid_argument("CLI requires run identity, positive gap and an explicit output directory");
    for(const auto& path:{result.source.canonical,result.source.scope,result.source.member,result.source.declarations,
        result.source.glass_resolution,result.source.type13,result.source.auxiliary_member,
        result.source.original_wall_member,result.source.wall_manifest}) {
        if(path.empty()) throw std::invalid_argument("CLI requires every explicit original source path");
    }
    return result;
}
const char* Usage() noexcept {
    return "robo_dyna_vehicle_run --canonical DIR --scope FILE --member FILE --declarations FILE "
        "--glass-resolution FILE --type13 FILE --aux-member FILE --original-wall-member FILE "
        "--wall-manifest FILE --run-id UINT --output EMPTY_DIR "
        "[--physical-profile retained-shell-v1|extended-solids-v4|vehicle-supports-v5] "
        "[--contact-profile wall-only|wall-self-contact-v1] [--self-contact-member FILE] "
        "[--duration-ms 0.5|5|20|50] [--fixed-dt-s 3e-7] [--gap-m .02] [--samples 101] "
        "[--wall-stiffness-n-m3 VALUE] [--penetration-limit-m VALUE] "
        "[--self-contact-failure-output ABSENT_DIR] [--diagnostic-intervals N] [--maximum-elapsed-s SEC] [--stop-file PATH] [--forecast-only] [--conditional-full-limits]";
}
} // namespace crash::cases::vehicle_run::cli
