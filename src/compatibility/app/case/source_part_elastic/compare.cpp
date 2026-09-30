#include "output/AcceptedReplay.h"
#include "output/ArtifactIO.h"
#include "output/SourcePartComparison.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
namespace io=crash::output;
using io::Require;
using io::Document;
using io::Value;
Document Read(const std::filesystem::path& path) {
    const auto text=io::ReadBounded(path,256*1024);
    Document d;d.Parse<rapidjson::kParseFullPrecisionFlag>(text.data(),text.size());
    Require(!d.HasParseError()&&d.IsObject(),"Invalid comparison document");return d;
}
const Value& Get(const Value& d,const char* name) {
    Require(d.IsObject()&&d.HasMember(name),"Missing comparison field");return d[name];
}
double Real(const Value& d,const char* name) {
    const auto& v=Get(d,name);Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Nonfinite comparison field");return v.GetDouble();
}
std::string Encode(const Value& v) {
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    v.Accept(writer);return {buffer.GetString(),buffer.GetSize()};
}
Document Fields(const std::filesystem::path& dir,std::uint64_t epoch) {
    std::ostringstream name;name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<epoch<<".fields.json";
    return Read(dir/name.str());
}
constexpr std::size_t Channels=6;
using Differences=std::array<double,Channels>;
const char* Names[]{"position","rotation","synchronized_velocity","synchronized_omega","total_energy","external_work"};
Differences Difference(const Value& a,const Value& b) {
    Require(io::Bits(Real(a,"accepted_time_s"))==io::Bits(Real(b,"accepted_time_s")),"Different physical sample times");
    const double ea=Real(a,"synchronized_kinetic_J")+Real(a,"total_internal_work_J");
    const double eb=Real(b,"synchronized_kinetic_J")+Real(b,"total_internal_work_J");
    const auto kinematics=io::source_comparison::Kinematics(a,b);
    return {kinematics[0],kinematics[1],kinematics[2],kinematics[3],
        std::abs(ea-eb)/.1,std::abs(Real(a,"external_drift_work_J")-Real(b,"external_drift_work_J"))/.1};
}
}
int main(int argc,char** argv) {
    Document result;result.SetObject();
    try {
        Require(argc==5,"usage: robo_dyna_source_part_compare H_DIR H2_DIR H4_DIR NEW_JSON");
        std::array<io::AcceptedReplay,3> readers;
        std::array<Document,3> config;
        const unsigned refinements[]{1,2,4};
        for(unsigned r=0;r<3;++r) {
            const auto opened=readers[r].Open(argv[r+1]);
            if(opened.status!=io::ReplayStatus::Ok) throw std::runtime_error(opened.diagnostic);
            const auto& info=*readers[r].info();
            Require(info.kind==io::ReplayKind::SourcePartElastic&&info.final_epoch==32768*refinements[r]&&
                io::Bits(info.final_time)==io::Bits(.001953125),"Comparison requires the full frozen source-part horizon");
            config[r]=Read(std::filesystem::path(argv[r+1])/"configuration.json");
            Require(io::Bits(Real(config[r],"fixed_dt_s"))==io::Bits(0x1p-24/refinements[r]),"Unexpected refinement step");
            for(const char* key:{"configuration_id","qualification_id","source_readiness_sha256","source_part_id","young_modulus_Pa","poisson_ratio", "density_kg_m3",
                "thickness_m","pulse_duration_s","acceleration_m_s2","spatial_axis","direction_xyz","reference_nodes","source_parents",
                "material_policy","attachment_policy","maximum_displacement_m","maximum_rotation_rad","maximum_strain",
                "maximum_thickness_curvature","minimum_area_ratio","maximum_area_ratio","minimum_thickness_ratio",
                "maximum_thickness_ratio","maximum_energy_residual_J","relative_energy_residual","maximum_native_dt_fraction"})
                Require(Encode(Get(config[0],key))==Encode(Get(config[r],key)),"Refinements changed the physical experiment");
        }
        for(const auto& item:std::array<std::pair<const char*,double>,15>{{
            {"acceleration_m_s2",10000},{"pulse_duration_s",4096*0x1p-24},{"spatial_axis",2},
            {"young_modulus_Pa",200e9},{"poisson_ratio",.3},{"maximum_displacement_m",.02},
            {"maximum_rotation_rad",.25},{"maximum_strain",.005},{"maximum_thickness_curvature",.01},
            {"minimum_area_ratio",.99},{"maximum_area_ratio",1.01},{"minimum_thickness_ratio",.99},
            {"maximum_thickness_ratio",1.01},{"maximum_energy_residual_J",1e-10},{"relative_energy_residual",.05}}})
            Require(io::Bits(Real(config[0],item.first))==io::Bits(item.second),"Run differs from frozen pilot tuple");
        Require(Real(config[0],"maximum_native_dt_fraction")==.125,"Run changed native timestep guard");
        const auto& direction=Get(config[0],"direction_xyz");
        Require(direction.IsArray()&&direction.Size()==3&&direction[0].GetDouble()==1&&
            direction[1].GetDouble()==0&&direction[2].GetDouble()==0,"Run changed frozen pulse direction");
        Differences coarse{},fine{};std::array<double,3> chord{};
        for(std::uint64_t step=0;step<=32768;step+=128) {
            std::array<Document,3> fields;
            for(unsigned r=0;r<3;++r) {
                fields[r]=Fields(argv[r+1],step*refinements[r]);
                chord[r]=std::max(chord[r],Real(fields[r],"maximum_chord_change_m"));
            }
            const auto a=Difference(fields[0],fields[1]),b=Difference(fields[1],fields[2]);
            for(unsigned c=0;c<Channels;++c){coarse[c]=std::max(coarse[c],a[c]);fine[c]=std::max(fine[c],b[c]);}
        }
        bool passed=true;Value channels(rapidjson::kArrayType);
        for(unsigned c=0;c<Channels;++c) {
            const bool valid=coarse[c]<=.10&&fine[c]<=.05&&fine[c]<=.8*coarse[c]+1e-6;
            Value item(rapidjson::kObjectType);item.AddMember("field",Value(Names[c],result.GetAllocator()),result.GetAllocator());
            item.AddMember("coarse_medium",coarse[c],result.GetAllocator());item.AddMember("medium_fine",fine[c],result.GetAllocator());
            item.AddMember("passed",valid,result.GetAllocator());channels.PushBack(item,result.GetAllocator());passed&=valid;
        }
        for(double value:chord)passed&=value>=1e-5;
        result.AddMember("channels",channels,result.GetAllocator());io::FiniteArray(result,"maximum_chord_change_m",chord.data(),3);
        io::String(result,"schema","robo_dyna.source_part_elastic_comparison.v1");
        io::String(result,"status",passed?"passed":"failed");io::Integer(result,"common_samples",257);
        for(unsigned r=0;r<3;++r)io::String(result,("manifest_sha256_"+std::to_string(refinements[r])).c_str(),
            io::Sha256(io::ReadBounded(std::filesystem::path(argv[r+1])/"manifest.json",1024*1024)));
        io::WriteJson(argv[4],result);
        std::cout<<(passed?"Source-part refinement and deformation passed":"Source-part comparison failed")<<'\n';return passed?0:1;
    } catch(const std::exception& e) {
        std::cerr<<"Source comparison incomplete: "<<e.what()<<'\n';return 1;
    }
}
