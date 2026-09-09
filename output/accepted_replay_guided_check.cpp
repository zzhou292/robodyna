#include "AcceptedReplay.h"
#include "ArtifactIO.h"
#include "ContactIntegrationMetadata.h"
#include "GuidedExperimentMetadata.h"
#include "CsvLedgerSegments.h"
#include "MeshArchive.h"
#include "case/CanonicalWallArtifacts.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace crash::output {
namespace {
namespace fs=std::filesystem;
namespace experiment=guided_experiment_metadata;
// Format/association fixture, not a new physical trajectory oracle. The wall
// is the exact real canonical asset; synthetic moving fields deliberately have
// simple independent values so identity/phase corruption is easy to isolate.
class GuidedBundle {
 public:
    fs::path directory;
    GuidedBundle() {
        const char* asset=std::getenv("ROBO_DYNA_CANONICAL_WALL");
        Require(asset && *asset,"Guided reader tests require ROBO_DYNA_CANONICAL_WALL");
        const auto bytes=case_data::ReadPinnedWallManifest(asset);
        case_data::CanonicalWall wall; std::istringstream input(bytes);
        Require(wall.Load(input).status==case_data::WallStatus::Ok,"Cannot load original test wall");
        auto pattern=(fs::temp_directory_path()/"guided-replay-XXXXXX").string();
        std::vector<char> name(pattern.begin(),pattern.end()); name.push_back(0);
        const auto created=::mkdtemp(name.data()); Require(created,"Cannot create guided replay test directory");
        directory=created;
        try {
            case_data::WriteCanonicalWallArtifacts(directory,wall,bytes);
            WriteJson(directory/"configuration.json",Configuration());
            for (unsigned i=0;i<3;++i) {
                const auto mesh=Mesh(i); WriteMeshFiles(directory,Stem(i),mesh);
                WriteJson(directory/(Stem(i)+".fields.json"),Fields(i,mesh));
            }
            WriteBytes(directory/"accepted-frames.csv",
                "owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n"
                "7,0,0,accepted-000000.mesh.json,accepted-000000.obj\n"
                "7,5,0.5,accepted-000005.mesh.json,accepted-000005.obj\n"
                "7,10,1,accepted-000010.mesh.json,accepted-000010.obj\n");
            for (const char* file : {"accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv"})
                WriteBytes(directory/file,"fixture ledger; physics is not requalified by this reader\n");
            Document final; final.SetObject(); Integer(final,"owner_id",7); Integer(final,"accepted_epoch",10);
            Number(final,"accepted_time_s",1); Integer(final,"saved_frames",3);
            WriteJson(directory/"final-metrics.json",final); Manifest();
        } catch (...) { std::error_code error; fs::remove_all(directory,error); throw; }
    }
    ~GuidedBundle() { std::error_code error; fs::remove_all(directory,error); }
    static std::string Stem(unsigned i) { return i==0 ? "accepted-000000" : i==1 ? "accepted-000005" : "accepted-000010"; }
    static chrono::ChTriangleMeshConnected Mesh(unsigned i) {
        chrono::ChTriangleMeshConnected mesh;
        const double x=.045+.001*i;
        mesh.GetCoordsVertices()={{x,.1,0},{x,0,0},{x,0,.1},{x,.1,.1},{x,.2,0},{x,.2,.1}};
        mesh.GetIndicesVertices()={{0,1,2},{0,2,3},{4,0,3},{4,3,5}}; return mesh;
    }
    static Value Zeros(Document& d,unsigned count) {
        Value a(rapidjson::kArrayType); for(unsigned i=0;i<count;++i)a.PushBack(0.,d.GetAllocator()); return a;
    }
    static Value Matrix(Document& d,unsigned rows,unsigned columns) {
        Value a(rapidjson::kArrayType); for(unsigned i=0;i<rows;++i)a.PushBack(Zeros(d,columns),d.GetAllocator()); return a;
    }
    static Value Parent(Document& d,unsigned e) {
        Value parent(rapidjson::kObjectType),nodes(rapidjson::kArrayType);
        const unsigned q[2][4]={{0,1,2,3},{4,0,3,5}};
        for(unsigned n:q[e])nodes.PushBack(n,d.GetAllocator());
        parent.AddMember("parent_element",101+e,d.GetAllocator()); parent.AddMember("parent_face",0,d.GetAllocator());
        parent.AddMember("feature_id",1001+e,d.GetAllocator());
        parent.AddMember("connectivity_zero_based",nodes,d.GetAllocator()); return parent;
    }
    static Document Configuration() {
        Document d; d.SetObject(); String(d,"schema","robo_dyna.guided_plate_configuration.v1");
        Number(d,"fixed_dt_s",.1); Number(d,"requested_horizon_s",1); Integer(d,"required_steps",10);
        Integer(d,"owner_id",7); Integer(d,"run_id",13); Integer(d,"topology_id",17);
        Integer(d,"qualification_id",experiment::OriginalQualification); Integer(d,"wall_binding_id",23);
        Number(d,"penalty_per_area_N_per_m3",experiment::Penalty(experiment::Original));
        Number(d,"maximum_penetration_m",experiment::MaximumPenetration);
        Number(d,"contact_force_error_budget_N",experiment::ForceError);
        Number(d,"contact_potential_error_budget_J",experiment::PotentialError);
        String(d,"canonical_wall_manifest_sha256",case_data::kCanonicalWallManifestSha256);
        Value vertices(rapidjson::kArrayType),triangles(rapidjson::kArrayType),parents(rapidjson::kArrayType);
        for(unsigned n=0;n<6;++n) {
            Value row(rapidjson::kArrayType);
            for(unsigned v:{n,1u,1u,n+1})row.PushBack(v,d.GetAllocator()); vertices.PushBack(row,d.GetAllocator());
        }
        const auto mesh=Mesh(0);
        for(unsigned i=0;i<4;++i) {
            const auto& face=mesh.GetIndicesVertices()[i]; Value row(rapidjson::kArrayType);
            for(unsigned v:{unsigned(face[0]),unsigned(face[1]),unsigned(face[2]),1u,1u,101+i/2,1u,0u,i%2})
                row.PushBack(v,d.GetAllocator());
            triangles.PushBack(row,d.GetAllocator());
        }
        for(unsigned e=0;e<2;++e)parents.PushBack(Parent(d,e),d.GetAllocator());
        d.AddMember("vertex_binding",vertices,d.GetAllocator()); d.AddMember("triangle_binding",triangles,d.GetAllocator());
        d.AddMember("contact_parent_binding",parents,d.GetAllocator()); return d;
    }
    static Document Fields(unsigned i,const chrono::ChTriangleMeshConnected& mesh) {
        Document d; d.SetObject(); String(d,"schema","robo_dyna.guided_plate_fields.v1");
        Integer(d,"owner_id",7); Integer(d,"accepted_epoch",5*i); Number(d,"accepted_time_s",.5*i);
        Number(d,"fixed_dt_s",.1); Boolean(d,"reactions_valid",i!=0);
        Integer(d,"reaction_base_epoch",i ? 5*i-1 : 0); Number(d,"reaction_time_s",i ? .5*i-.1 : 0);
        const unsigned base=i==1 ? 4 : 5*i,attempt=i==2 ? 21 : 1+4*i;
        const char* phase=i==1 ? "prepared_candidate_subsequently_committed" : "accepted_base";
        Integer(d,"element_evaluation_base_epoch",base); Integer(d,"element_evaluation_attempt",attempt);
        Integer(d,"contact_evaluation_base_epoch",base); Integer(d,"contact_evaluation_attempt",attempt);
        String(d,"element_evaluation_phase",phase); String(d,"contact_evaluation_phase",phase);
        Integer(d,"qualification_id",experiment::OriginalQualification); Integer(d,"wall_binding_id",23);
        Value positions(rapidjson::kArrayType),rotations(rapidjson::kArrayType);
        for(const auto& p:mesh.GetCoordsVertices()) {
            for(unsigned j=0;j<3;++j)positions.PushBack(p[j],d.GetAllocator());
            for(double q:{1.,0.,0.,0.})rotations.PushBack(q,d.GetAllocator());
        }
        d.AddMember("position_xyz_m",positions,d.GetAllocator()); d.AddMember("orientation_wxyz",rotations,d.GetAllocator());
        for(const char* key:{"velocity_xyz_m_per_s","omega_world_xyz_rad_per_s","reaction_force_xyz_N_at_base",
                             "reaction_couple_world_xyz_Nm_at_base"}) {
            Value name(key,d.GetAllocator()); d.AddMember(name,Zeros(d,18),d.GetAllocator());
        }
        Value elements(rapidjson::kArrayType),parents(rapidjson::kArrayType),contact(rapidjson::kObjectType);
        for(unsigned e=0;e<2;++e) {
            Value element(rapidjson::kObjectType); element.AddMember("parent_element",101+e,d.GetAllocator());
            element.AddMember("elastic_energy_J",0.,d.GetAllocator()); element.AddMember("bending_energy_J",0.,d.GetAllocator());
            element.AddMember("force_world_N",Matrix(d,4,3),d.GetAllocator());
            element.AddMember("couple_world_Nm",Matrix(d,4,3),d.GetAllocator());
            element.AddMember("gauss_strain",Matrix(d,4,12),d.GetAllocator());
            element.AddMember("gauss_resultant",Matrix(d,4,12),d.GetAllocator()); elements.PushBack(element,d.GetAllocator());
            auto parent=Parent(d,e); parent.AddMember("base_epoch",base,d.GetAllocator()); parent.AddMember("attempt",attempt,d.GetAllocator());
            parent.AddMember("covered",true,d.GetAllocator()); parent.AddMember("valid",true,d.GetAllocator());
            parent.AddMember("force_world_N",Matrix(d,4,3),d.GetAllocator());
            parent.AddMember("couple_world_Nm",Matrix(d,4,3),d.GetAllocator());
            parent.AddMember("nodal_force_magnitude_N",Matrix(d,4,4),d.GetAllocator());
            parent.AddMember("resultant_magnitude_N",Zeros(d,4),d.GetAllocator());
            parent.AddMember("potential_J",Zeros(d,4),d.GetAllocator()); parent.AddMember("active_area_m2",Zeros(d,2),d.GetAllocator());
            parent.AddMember("leaf_count",1,d.GetAllocator()); parent.AddMember("visited",1,d.GetAllocator());
            parent.AddMember("deepest_leaf",0,d.GetAllocator()); parents.PushBack(parent,d.GetAllocator());
        }
        for(const char* key:{"wall_reaction_N","wall_moment_Nm","force_error_N","wall_moment_error_Nm"}) {
            Value name(key,d.GetAllocator()); contact.AddMember(name,Zeros(d,3),d.GetAllocator());
        }
        contact.AddMember("potential_J",Zeros(d,4),d.GetAllocator()); contact.AddMember("active_area_m2",Zeros(d,2),d.GetAllocator());
        contact.AddMember("maximum_penetration_m",0.,d.GetAllocator());
        d.AddMember("elements",elements,d.GetAllocator()); d.AddMember("contact_parents",parents,d.GetAllocator());
        d.AddMember("contact_result",contact,d.GetAllocator()); return d;
    }
    Document Read(const std::string& file) const {
        const auto bytes=ReadBounded(directory/file,32*1024*1024); Document d;
        d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size()); Require(!d.HasParseError(),"Bad fixture JSON"); return d;
    }
    void Replace(const std::string& file,const Document& d) { fs::remove(directory/file); WriteJson(directory/file,d); }
    void Replace(const std::string& file,const std::string& bytes) { fs::remove(directory/file); WriteBytes(directory/file,bytes); }
    void Backend(const char* backend) {
        auto config=Read("configuration.json"); String(config,contact_metadata::BackendField,backend);
        if(!config.HasMember("contact_max_depth"))Integer(config,"contact_max_depth",16);
        if(!config.HasMember("contact_max_leaves"))Integer(config,"contact_max_leaves",4096);
        if(!config.HasMember("contact_max_visits"))Integer(config,"contact_max_visits",16384);
        Replace("configuration.json",config);
        for(unsigned i=0;i<3;++i) {
            const auto file=Stem(i)+".fields.json"; auto d=Read(file);
            String(d,contact_metadata::BackendField,backend);
            const unsigned v=std::string_view(backend)==contact_metadata::Scalar?2:1;
            for(auto& parent:d["contact_parents"].GetArray()) {
                parent.AddMember(rapidjson::StringRef(contact_metadata::BackendField),Value(backend,d.GetAllocator()),d.GetAllocator());
                parent["deepest_leaf"].SetUint(2);
                parent.AddMember("deepest_u",2,d.GetAllocator()); parent.AddMember("deepest_v",v,d.GetAllocator());
            }
            auto& c=d["contact_result"];
            c.AddMember(rapidjson::StringRef(contact_metadata::BackendField),Value(backend,d.GetAllocator()),d.GetAllocator());
            c.AddMember("deepest_leaf",2,d.GetAllocator()); c.AddMember("deepest_u",2,d.GetAllocator()); c.AddMember("deepest_v",v,d.GetAllocator());
            Replace(file,d);
        }
        auto final=Read("final-metrics.json"); String(final,contact_metadata::BackendField,backend); Replace("final-metrics.json",final);
        Manifest();
    }
    void Experiment(const char* name) {
        auto config=Read("configuration.json"); String(config,experiment::Field,name);
        config["qualification_id"].SetUint64(experiment::Qualification(name));
        config["penalty_per_area_N_per_m3"].SetDouble(experiment::Penalty(name));
        Number(config,"penalty_target_penetration_m",experiment::TargetPenetration);
        if(!config.HasMember("contact_max_depth"))Integer(config,"contact_max_depth",experiment::MaxDepth);
        if(!config.HasMember("contact_max_leaves"))Integer(config,"contact_max_leaves",experiment::MaxLeaves);
        if(!config.HasMember("contact_max_visits"))Integer(config,"contact_max_visits",experiment::MaxVisits);
        Replace("configuration.json",config);
        for(unsigned i=0;i<3;++i) {
            const auto file=Stem(i)+".fields.json"; auto d=Read(file);
            String(d,experiment::Field,name); d["qualification_id"].SetUint64(experiment::Qualification(name)); Replace(file,d);
        }
        auto final=Read("final-metrics.json"); String(final,experiment::Field,name);
        Integer(final,"qualification_id",experiment::Qualification(name)); Replace("final-metrics.json",final); Manifest();
    }
    // Small-memory format fixture: conservative 1664-byte maximum rows force
    // an actual two-file plan under the UNCHANGED 32MiB cap, but each manually
    // written seven-column row is short. This is metadata/row validation, not
    // an executed long physical trajectory or use of the segmented writer.
    void Segmented(std::uint64_t intervals=21000) {
        const double dt=1./static_cast<double>(intervals);
        std::vector<double> time(intervals+1);
        for(std::uint64_t e=1;e<=intervals;++e)time[e]=time[e-1]+dt;
        const std::array<std::uint64_t,3> epochs{{0,intervals/2,intervals}};
        const auto stem=[](std::uint64_t e) {
            std::ostringstream out; out<<"accepted-"<<std::setw(6)<<std::setfill('0')<<e; return out.str();
        };
        std::ostringstream index; index<<std::setprecision(17)
            <<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
        for(unsigned i=0;i<3;++i) {
            const auto epoch=epochs[i]; const auto old=Stem(i),now=stem(epoch);
            auto fields=Read(old+".fields.json");
            fields["accepted_epoch"].SetUint64(epoch); fields["accepted_time_s"].SetDouble(time[epoch]);
            fields["fixed_dt_s"].SetDouble(dt);
            fields["reaction_base_epoch"].SetUint64(epoch?epoch-1:0);
            fields["reaction_time_s"].SetDouble(epoch?time[epoch-1]:0);
            const auto base=i==1?epoch-1:epoch,attempt=i==2?epoch+2:epoch+1;
            fields["element_evaluation_base_epoch"].SetUint64(base);
            fields["contact_evaluation_base_epoch"].SetUint64(base);
            fields["element_evaluation_attempt"].SetUint64(attempt);
            fields["contact_evaluation_attempt"].SetUint64(attempt);
            for(auto& parent:fields["contact_parents"].GetArray()) {
                parent["base_epoch"].SetUint64(base); parent["attempt"].SetUint64(attempt);
            }
            if(now!=old) for(const char* suffix:{".mesh.json",".obj"})fs::rename(directory/(old+suffix),directory/(now+suffix));
            fs::remove(directory/(old+".fields.json")); WriteJson(directory/(now+".fields.json"),fields);
            index<<7<<','<<epoch<<','<<time[epoch]<<','<<now<<".mesh.json,"<<now<<".obj\n";
        }
        Replace("accepted-frames.csv",index.str());
        std::array<CsvLedgerPlan,3> plans;
        const std::array<const char*,3> names{{"accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv"}};
        const std::string header="owner_id,base_epoch,attempt,force_eval_time_s,accepted_epoch,accepted_time_s,value_J\n";
        for(std::size_t ledger=0;ledger<3;++ledger) {
            plans[ledger]=PlanCsvLedger(names[ledger],header,intervals,kCsvLedgerRowCap);
            fs::remove(directory/names[ledger]);
            for(const auto& segment:plans[ledger].segments) {
                std::ostringstream rows; rows<<std::setprecision(17)<<header;
                for(auto e=segment.first_epoch;e<=segment.last_epoch;++e)
                    rows<<7<<','<<e-1<<','<<e+1<<','<<time[e-1]<<','<<e<<','<<time[e]<<','<<ledger+1<<'\n';
                WriteBytes(directory/segment.file,rows.str());
            }
        }
        auto config=Read("configuration.json"); config["required_steps"].SetUint64(intervals); config["fixed_dt_s"].SetDouble(dt);
        AppendCsvLedgerSegments(config,plans.data(),plans.size()); Replace("configuration.json",config);
        auto final=Read("final-metrics.json"); final["accepted_epoch"].SetUint64(intervals);
        final["accepted_time_s"].SetDouble(time.back()); Replace("final-metrics.json",final); Manifest();
    }
    void Manifest() {
        fs::remove(directory/"manifest.json"); Document d; d.SetObject();
        const auto config=Read("configuration.json"),final=Read("final-metrics.json");
        String(d,"schema",config.HasMember(kCsvLedgerSegmentsField)?"robo_dyna.guided_plate_artifacts.v2":
                                                                  "robo_dyna.guided_plate_artifacts.v1");
        String(d,"status","completed");
        Boolean(d,"shell_model",true); Boolean(d,"vehicle_model",false); Boolean(d,"contact",true);
        Integer(d,"owner_id",7); Integer(d,"accepted_epoch",final["accepted_epoch"].GetUint64());
        Number(d,"accepted_time_s",final["accepted_time_s"].GetDouble());
        if(config.HasMember(kCsvLedgerSegmentsField))
            d.AddMember(rapidjson::StringRef(kCsvLedgerSegmentsField),
                        Value(config[kCsvLedgerSegmentsField],d.GetAllocator()),d.GetAllocator());
        if(config.HasMember(contact_metadata::BackendField))
            d.AddMember(rapidjson::StringRef(contact_metadata::BackendField),
                        Value(config[contact_metadata::BackendField],d.GetAllocator()),d.GetAllocator());
        if(config.HasMember(experiment::Field)) {
            d.AddMember(rapidjson::StringRef(experiment::Field),Value(config[experiment::Field],d.GetAllocator()),d.GetAllocator());
            Integer(d,"qualification_id",config["qualification_id"].GetUint64());
        }
        Value inventory(rapidjson::kArrayType);
        for(const auto& file:fs::directory_iterator(directory)) {
            const auto name=file.path().filename().string(), bytes=ReadBounded(file.path(),32*1024*1024),hash=Sha256(bytes);
            Value row(rapidjson::kObjectType); row.AddMember("file",Value(name.c_str(),d.GetAllocator()),d.GetAllocator());
            row.AddMember("sha256",Value(hash.c_str(),d.GetAllocator()),d.GetAllocator());
            row.AddMember("bytes",Value().SetUint64(bytes.size()),d.GetAllocator()); inventory.PushBack(row,d.GetAllocator());
        }
        d.AddMember("artifacts",inventory,d.GetAllocator()); WriteJson(directory/"manifest.json",d);
    }
};

TEST(AcceptedReplayGuided, OriginalWallAndCandidateOrRefreshedEndpointFramesAreBound) {
    GuidedBundle f;
    // A rounded estimate just below a degenerate truth enclosure is legal
    // when its declared radius covers the gap; inclusion is not required.
    const auto name=GuidedBundle::Stem(2)+".fields.json"; auto rounded=f.Read(name);
    auto& certificate=rounded["contact_parents"][0]["resultant_magnitude_N"];
    const double estimate=std::nextafter(1.,0.);
    certificate[0].SetDouble(estimate); certificate[1].SetDouble(1.);
    certificate[2].SetDouble(1.); certificate[3].SetDouble(1.-estimate);
    f.Replace(name,rounded); f.Manifest();
    AcceptedReplay replay; const auto result=replay.Open(f.directory);
    ASSERT_EQ(result.status,ReplayStatus::Ok)<<result.diagnostic;
    EXPECT_EQ(replay.info()->kind,ReplayKind::GuidedPlate); EXPECT_EQ(replay.info()->run_id,13u);
    EXPECT_EQ(replay.info()->guided_experiment,experiment::Original);
    EXPECT_EQ(replay.info()->topology_id,17u); ASSERT_TRUE(replay.wall());
    EXPECT_EQ(replay.wall()->GetCoordsVertices().size(),62u); EXPECT_EQ(replay.wall()->GetIndicesVertices().size(),100u);
    for(unsigned i:{2u,1u,0u}) {
        const auto load=replay.Load(i); ASSERT_EQ(load.status,ReplayStatus::Ok)<<load.diagnostic;
        EXPECT_EQ(replay.frame()->epoch,5*i); EXPECT_EQ(Bits(replay.frame()->time),Bits(.5*i));
        EXPECT_EQ(replay.frame()->mesh->GetCoordsVertices(),GuidedBundle::Mesh(i).GetCoordsVertices());
    }
}

TEST(AcceptedReplayGuided, AllLedgersCanonicalFilesAndScopeFlagsAreRequired) {
    for(const char* file:{"accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv",
                          "canonical-wall.manifest.json","canonical-wall.mesh.json","canonical-wall.obj"}) {
        GuidedBundle f; fs::remove(f.directory/file); f.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle)<<file;
    }
    for(const char* flag:{"shell_model","vehicle_model","contact"}) {
        GuidedBundle f; auto d=f.Read("manifest.json"); d[flag].SetBool(!d[flag].GetBool()); f.Replace("manifest.json",d);
        AcceptedReplay replay; EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle)<<flag;
    }
}

TEST(AcceptedReplayGuided, RehashedWallSubstitutionsCannotPassTheSourcePinOrExactMesh) {
    for(unsigned fault=0;fault<4;++fault) {
        GuidedBundle f;
        if(fault==0) {
            const auto bytes=ReadBounded(f.directory/"canonical-wall.manifest.json",1024*1024);
            f.Replace("canonical-wall.manifest.json",bytes+"\n");
        } else if(fault==1) {
            auto d=f.Read("configuration.json"); d["canonical_wall_manifest_sha256"].SetString(std::string(64,'0').c_str(),d.GetAllocator());
            f.Replace("configuration.json",d);
        } else {
            auto d=f.Read("canonical-wall.mesh.json");
            if(fault==2)for(auto& p:d["mesh"]["m_vertices"].GetArray())p["x"].SetDouble(.051);
            else { auto& t=d["mesh"]["m_face_v_indices"][0]; const auto a=t["x"].GetInt(); t["x"].SetInt(t["y"].GetInt());t["y"].SetInt(a); }
            f.Replace("canonical-wall.mesh.json",d);
        }
        f.Manifest(); AcceptedReplay replay; EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle)<<fault;
    }
}

TEST(AcceptedReplayGuided, ConfigurationAndBothContributorParentBindingsAreImmutable) {
    for(unsigned fault=0;fault<8;++fault) {
        GuidedBundle f; const auto name=fault<4 ? "configuration.json" : GuidedBundle::Stem(2)+".fields.json";
        auto d=f.Read(name);
        if(fault==0)d["contact_parent_binding"][1]["feature_id"].SetUint64(1001);
        if(fault==1)d["contact_parent_binding"][1]["connectivity_zero_based"][3].SetUint64(99);
        if(fault==2)d["triangle_binding"][2][5].SetUint64(101);
        if(fault==3)d["contact_parent_binding"].PopBack();
        if(fault==4)d["elements"][1]["parent_element"].SetUint64(101);
        if(fault==5)d["contact_parents"][1]["feature_id"].SetUint64(1001);
        if(fault==6)d["contact_parents"][1]["connectivity_zero_based"][3].SetUint64(2);
        if(fault==7)d["contact_parents"][1]["parent_face"].SetUint64(1);
        f.Replace(name,d); f.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle)<<fault;
    }
}

TEST(AcceptedReplayGuided, AcceptedAndBasePhasesCannotBeRelabeledOrMixed) {
    for(unsigned fault=0;fault<11;++fault) {
        GuidedBundle f; const auto name=GuidedBundle::Stem(2)+".fields.json"; auto d=f.Read(name);
        if(fault==0)d["owner_id"].SetUint64(8);
        if(fault==1)d["qualification_id"].SetUint64(20);
        if(fault==2)d["wall_binding_id"].SetUint64(24);
        if(fault==3)d["element_evaluation_base_epoch"].SetUint64(9);
        if(fault==4)d["contact_evaluation_attempt"].SetUint64(22);
        if(fault==5)d["contact_evaluation_phase"].SetString("prepared_candidate_subsequently_committed",d.GetAllocator());
        if(fault==6)d["contact_parents"][1]["base_epoch"].SetUint64(9);
        if(fault==7)d["reaction_base_epoch"].SetUint64(10);
        if(fault==8)d["reaction_time_s"].SetDouble(1);
        if(fault==9)d["reactions_valid"].SetBool(false);
        if(fault==10)d["fixed_dt_s"].SetDouble(.2);
        f.Replace(name,d); f.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle)<<fault;
    }
}

TEST(AcceptedReplayGuided, CompleteFiniteFieldsAndPositionBitsAreRequired) {
    for(unsigned fault=0;fault<13;++fault) {
        GuidedBundle f; const auto name=GuidedBundle::Stem(2)+".fields.json"; auto d=f.Read(name);
        if(fault==0)d["orientation_wxyz"].PopBack();
        if(fault==1)d.RemoveMember("velocity_xyz_m_per_s");
        if(fault==2)d["position_xyz_m"][17].SetDouble(.2);
        if(fault==3)d["elements"][1]["gauss_resultant"][3].PopBack();
        if(fault==4)d["contact_parents"][1]["force_world_N"][3][2].SetString("NaN",d.GetAllocator());
        if(fault==5)d["contact_parents"][1]["potential_J"][1].SetDouble(1);
        if(fault==6)d["contact_result"]["active_area_m2"][0].SetDouble(-1);
        if(fault==7)d["contact_result"].RemoveMember("wall_moment_Nm");
        if(fault==8)d["contact_parents"][1]["valid"].SetBool(false);
        if(fault==9) { d["contact_parents"][1]["covered"].SetBool(false); d["contact_parents"][1]["valid"].SetBool(false); }
        if(fault==10)d["contact_result"]["wall_moment_error_Nm"][2].SetDouble(-1);
        if(fault==11)d["contact_parents"][1]["potential_J"][2].SetDouble(1); // Zero declared error cannot cover upper endpoint.
        if(fault==12) { auto& c=d["contact_parents"][1]["resultant_magnitude_N"]; c[1].SetDouble(-1); c[3].SetDouble(1); }
        f.Replace(name,d); f.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle)<<fault;
    }
}

TEST(AcceptedReplayGuided, FailedLateLoadAndOpenPreservePublishedReaderAndAllowExactRetry) {
    GuidedBundle f; AcceptedReplay replay; ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
    ASSERT_EQ(replay.Load(1).status,ReplayStatus::Ok); const auto frame=*replay.frame(); const auto wall=replay.wall();
    const auto name=GuidedBundle::Stem(2)+".fields.json"; const auto bytes=ReadBounded(f.directory/name,32*1024*1024);
    auto bad=f.Read(name); bad["contact_evaluation_attempt"].SetUint64(99); f.Replace(name,bad);
    EXPECT_EQ(replay.Load(2).status,ReplayStatus::InvalidFrame); EXPECT_EQ(replay.frame()->mesh,frame.mesh);
    f.Manifest(); EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle);
    EXPECT_EQ(replay.frame()->epoch,frame.epoch); EXPECT_EQ(replay.frame()->mesh,frame.mesh); EXPECT_EQ(replay.wall(),wall);
    f.Replace(name,bytes); ASSERT_EQ(replay.Load(2).status,ReplayStatus::Ok);
    EXPECT_EQ(replay.frame()->epoch,10u); f.Manifest(); EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
}

TEST(AcceptedReplayGuided, ExplicitScalarAndRectangularPartitionsRemainReadable) {
    for(const char* backend:{contact_metadata::Scalar,contact_metadata::Rectangular}) {
        GuidedBundle f; f.Backend(backend); AcceptedReplay replay;
        const auto open=replay.Open(f.directory); ASSERT_EQ(open.status,ReplayStatus::Ok)<<open.diagnostic;
        for(unsigned i=0;i<3;++i)EXPECT_EQ(replay.Load(i).status,ReplayStatus::Ok);
    }
}

TEST(AcceptedReplayGuided, NamedExperimentsBindQualificationWithoutChangingReplayGeometry) {
    for(const char* name:{experiment::Original,experiment::PenaltyMargin})
        for(const char* backend:{contact_metadata::Scalar,contact_metadata::Rectangular}) {
            SCOPED_TRACE(name);
            SCOPED_TRACE(backend);
            GuidedBundle f; f.Experiment(name); f.Backend(backend); AcceptedReplay replay;
            const auto opened=replay.Open(f.directory); ASSERT_EQ(opened.status,ReplayStatus::Ok)<<opened.diagnostic;
            EXPECT_EQ(replay.info()->guided_experiment,name);
            for(unsigned i=0;i<3;++i) {
                ASSERT_EQ(replay.Load(i).status,ReplayStatus::Ok);
                EXPECT_EQ(replay.frame()->mesh->GetCoordsVertices(),GuidedBundle::Mesh(i).GetCoordsVertices());
                EXPECT_EQ(Bits(replay.frame()->time),Bits(.5*i));
            }
        }
    // An optional declaration on a legacy record must agree with its default.
    GuidedBundle legacy; auto final=legacy.Read("final-metrics.json");
    String(final,experiment::Field,experiment::Original); legacy.Replace("final-metrics.json",final); legacy.Manifest();
    AcceptedReplay replay; ASSERT_EQ(replay.Open(legacy.directory).status,ReplayStatus::Ok);
    EXPECT_EQ(replay.info()->guided_experiment,experiment::Original);
}

TEST(AcceptedReplayGuided, NamedExperimentCorruptionPreservesPreviouslyPublishedReplay) {
    for(unsigned fault=0;fault<38;++fault) {
        SCOPED_TRACE(fault); GuidedBundle f;
        const bool legacy=fault>=23&&fault<=28;
        if(!legacy)f.Experiment(experiment::PenaltyMargin);
        AcceptedReplay replay; ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
        ASSERT_EQ(replay.Load(1).status,ReplayStatus::Ok);
        const auto previous=*replay.frame(); const auto info=*replay.info(); const auto wall=replay.wall();
        std::string file="configuration.json";
        if((fault>=10&&fault<=13)||fault==25)file=GuidedBundle::Stem(2)+".fields.json";
        if(fault==14||fault==15||fault==18||fault==21||fault==26)file="final-metrics.json";
        if(fault==16||fault==17||fault==22||fault==27)file="manifest.json";
        auto d=f.Read(file);
        if(fault==0)d[experiment::Field].SetString("unknown");
        if(fault==1)d[experiment::Field].SetUint(1);
        if(fault==2||fault==13||fault==21||fault==22)String(d,experiment::Field,experiment::PenaltyMargin);
        if(fault==3) {
            constexpr char bad[]="penalty-margin-v1\0extra";
            d[experiment::Field].SetString(bad,sizeof(bad)-1,d.GetAllocator());
        }
        if(fault==4||fault==12||fault==18)d["qualification_id"].SetUint64(experiment::OriginalQualification);
        if(fault==5)d["penalty_per_area_N_per_m3"].SetDouble(1e5);
        if(fault==6)d["maximum_penetration_m"].SetDouble(.001);
        if(fault==7)d["contact_force_error_budget_N"].SetDouble(2e-6);
        if(fault==8)d["contact_potential_error_budget_J"].SetDouble(2e-12);
        if(fault==9)d.RemoveMember("contact_force_error_budget_N");
        if(fault==10||fault==14||fault==16)d[experiment::Field].SetString(experiment::Original,d.GetAllocator());
        if(fault==11||fault==15||fault==17||fault==20)d.RemoveMember(experiment::Field);
        if(fault==19)d["penalty_target_penetration_m"].SetDouble(.001);
        if(fault==23)d["qualification_id"].SetUint64(experiment::PenaltyMarginQualification);
        if(fault==24)d["penalty_per_area_N_per_m3"].SetDouble(4e5);
        if(fault==25||fault==26||fault==27)String(d,experiment::Field,experiment::PenaltyMargin);
        if(fault==28)d["contact_force_error_budget_N"].SetString("NaN");
        if(fault==29)d.RemoveMember("penalty_target_penetration_m");
        if(fault==30)d["contact_max_leaves"].SetUint(2048);
        if(fault==31)d["contact_max_visits"].SetUint(8192);
        if(fault==32)d["contact_max_depth"].SetUint(15);
        if(fault==33)Integer(d,"contact_max_leaves",experiment::MaxLeaves);
        if(fault==34)Number(d,"penalty_per_area_N_per_m3",4e5);
        if(fault==35)Integer(d,"qualification_id",experiment::PenaltyMarginQualification);
        if(fault==36)Number(d,"contact_potential_error_budget_J",experiment::PotentialError);
        if(fault==37)Number(d,"penalty_target_penetration_m",experiment::TargetPenetration);
        f.Replace(file,d); if(file!="manifest.json")f.Manifest();
        const auto rejected=replay.Open(f.directory);
        EXPECT_EQ(rejected.status,ReplayStatus::InvalidBundle)<<rejected.diagnostic;
        EXPECT_EQ(replay.info()->guided_experiment,info.guided_experiment);
        EXPECT_EQ(replay.info()->owner_id,info.owner_id); EXPECT_EQ(replay.info()->final_epoch,info.final_epoch);
        EXPECT_EQ(Bits(replay.info()->final_time),Bits(info.final_time));
        EXPECT_EQ(replay.frame()->mesh,previous.mesh); EXPECT_EQ(replay.frame()->epoch,previous.epoch);
        EXPECT_EQ(Bits(replay.frame()->time),Bits(previous.time)); EXPECT_EQ(replay.wall(),wall);
    }
}

TEST(AcceptedReplayGuided, ExperimentProtocolRejectsUnknownOrRepeatedConsumedValues) {
    Document legacy; legacy.SetObject(); EXPECT_EQ(experiment::Name(legacy),experiment::Original);
    EXPECT_THROW(experiment::Name(legacy,true),std::runtime_error);
    EXPECT_THROW(experiment::Qualification("unknown"),std::runtime_error);
    EXPECT_THROW(experiment::Penalty("unknown"),std::runtime_error);
    EXPECT_EQ(experiment::Qualification(experiment::Original),0x4432475549444531ULL);
    EXPECT_EQ(experiment::Qualification(experiment::PenaltyMargin),0x4432475549444532ULL);
    EXPECT_EQ(experiment::Penalty(experiment::Original),1e5); EXPECT_EQ(experiment::Penalty(experiment::PenaltyMargin),4e5);
    String(legacy,experiment::Field,experiment::Original); String(legacy,experiment::Field,experiment::Original);
    EXPECT_THROW(experiment::Name(legacy),std::runtime_error);
    Document scalar; scalar.SetInt(0); EXPECT_THROW(experiment::Name(scalar),std::runtime_error);
}

TEST(AcceptedReplayGuided, RehashedBackendAndAxisCorruptionRejectsWithoutReplacingPublishedFrames) {
    for(unsigned fault=0;fault<17;++fault) {
        SCOPED_TRACE(fault); GuidedBundle f;
        if(fault<10)f.Backend(contact_metadata::Rectangular);
        AcceptedReplay replay;
        ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok); ASSERT_EQ(replay.Load(1).status,ReplayStatus::Ok);
        const auto frame=*replay.frame(); const auto wall=replay.wall();
        const std::string file=fault==0?"configuration.json":fault==9?"final-metrics.json":GuidedBundle::Stem(2)+".fields.json";
        auto d=f.Read(file);
        if(fault==0)d[contact_metadata::BackendField].SetString("unknown");
        if(fault==1)d.RemoveMember(contact_metadata::BackendField);
        if(fault==2)d["contact_parents"][1][contact_metadata::BackendField].SetString(contact_metadata::Scalar,d.GetAllocator());
        if(fault==3)d["contact_parents"][1]["deepest_u"].SetUint(17);
        if(fault==4)d["contact_parents"][1]["deepest_leaf"].SetUint(3);
        if(fault==5)d["contact_result"]["deepest_v"].SetUint(0);
        if(fault==6)d["contact_parents"][1].RemoveMember("deepest_u");
        if(fault==7)String(d,contact_metadata::BackendField,contact_metadata::Rectangular);
        if(fault==8)d["contact_parents"][1].AddMember("deepest_v",1,d.GetAllocator());
        if(fault==9)d[contact_metadata::BackendField].SetString(contact_metadata::Scalar,d.GetAllocator());
        if(fault==10)d["contact_result"].AddMember(rapidjson::StringRef(contact_metadata::BackendField),
            Value(contact_metadata::Rectangular,d.GetAllocator()),d.GetAllocator());
        if(fault==11) {
            auto& c=d["contact_result"]; c.AddMember("deepest_leaf",0,d.GetAllocator());
            c.AddMember("deepest_u",17,d.GetAllocator()); c.AddMember("deepest_v",0,d.GetAllocator());
        }
        if(fault==12)d["contact_parents"][1]["leaf_count"].SetUint(0);
        if(fault==13)d["contact_parents"][1]["visited"].SetUint64(UINT64_MAX);
        if(fault==16)d["contact_result"].SetInt(0);
        if(fault==14||fault==15) {
            auto config=f.Read("configuration.json");
            if(fault==14) { Integer(config,"contact_max_depth",1); d["contact_parents"][1]["deepest_leaf"].SetUint(2); }
            if(fault==15) { Integer(config,"contact_max_visits",1); d["contact_parents"][1]["visited"].SetUint(2); }
            f.Replace("configuration.json",config);
        }
        f.Replace(file,d); f.Manifest();
        EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::InvalidBundle);
        EXPECT_EQ(replay.frame()->mesh,frame.mesh); EXPECT_EQ(replay.frame()->epoch,frame.epoch); EXPECT_EQ(replay.wall(),wall);
    }
}

void ExpectRejectedPreservingReplay(AcceptedReplay& replay,const fs::path& directory) {
    const auto frame=*replay.frame(); const auto wall=replay.wall(); const auto info=*replay.info();
    const auto report=replay.Open(directory);
    EXPECT_EQ(report.status,ReplayStatus::InvalidBundle)<<report.diagnostic;
    EXPECT_EQ(replay.frame()->mesh,frame.mesh); EXPECT_EQ(replay.frame()->epoch,frame.epoch);
    EXPECT_EQ(Bits(replay.frame()->time),Bits(frame.time)); EXPECT_EQ(replay.wall(),wall);
    EXPECT_EQ(replay.info()->schema,info.schema); EXPECT_EQ(replay.info()->final_epoch,info.final_epoch);
}

TEST(AcceptedReplayGuided, SegmentedV2ReplaysExactRowsAndPreservesLegacyV1) {
    GuidedBundle f; AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
    EXPECT_EQ(replay.info()->schema,"robo_dyna.guided_plate_artifacts.v1");
    f.Segmented();
    const auto report=replay.Open(f.directory); ASSERT_EQ(report.status,ReplayStatus::Ok)<<report.diagnostic;
    EXPECT_EQ(replay.info()->schema,"robo_dyna.guided_plate_artifacts.v2");
    EXPECT_EQ(replay.info()->frame_count,3u); EXPECT_EQ(replay.info()->final_epoch,21000u);
    const auto config=f.Read("configuration.json");
    const auto plans=ParseCsvLedgerSegments(config[kCsvLedgerSegmentsField]);
    ASSERT_EQ(plans.size(),3u);
    for(const auto& plan:plans) {
        EXPECT_EQ(plan.segments.size(),2u);
        EXPECT_LE(plan.segments.front().byte_cap,kArtifactFileCap);
        EXPECT_LT(fs::file_size(f.directory/plan.segments.front().file),2u*1024*1024);
    }
    for(const unsigned i:{1u,2u,0u}) {
        ASSERT_EQ(replay.Load(i).status,ReplayStatus::Ok);
        EXPECT_EQ(replay.frame()->epoch,10500u*i);
        EXPECT_EQ(replay.frame()->mesh->GetCoordsVertices(),GuidedBundle::Mesh(i).GetCoordsVertices());
    }
}

TEST(AcceptedReplayGuided, SegmentedMetadataIsBoundedDeterministicAndRepeatedExactly) {
    GuidedBundle f; f.Segmented(); AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok); ASSERT_EQ(replay.Load(2).status,ReplayStatus::Ok);
    const auto saved=f.Read("configuration.json");
    for(unsigned fault=0;fault<16;++fault) {
        Document config; config.CopyFrom(saved,config.GetAllocator());
        auto& plans=config[kCsvLedgerSegmentsField]; auto& plan=plans[2]; auto& segment=plan["segments"][1];
        switch(fault) {
            case 2: plan["logical_file"].SetString("other.csv",config.GetAllocator()); break;
            case 3: plan["interval_count"].SetUint64(20999); break;
            case 4: plan["header_sha256"].SetString(std::string(64,'a').c_str(),config.GetAllocator()); break;
            case 5: plan["column_count"].SetUint64(6); break;
            case 6: plan["max_row_bytes"].SetUint64(kCsvLedgerRowCap+1); break;
            case 7: segment["first_epoch"].SetUint64(segment["first_epoch"].GetUint64()-1); break;
            case 8: segment["row_count"].SetUint64(segment["row_count"].GetUint64()-1); break;
            case 9: segment["file"].SetString("../outside.csv",config.GetAllocator()); break;
            case 10: segment["byte_cap"].SetUint64(kArtifactFileCap+1); break;
            case 11: plan["logical_file"].SetString("accepted-intervals.csv",config.GetAllocator()); break;
            case 12:
                for(unsigned i=0;i<7;++i)plan["segments"].PushBack(Value(plan["segments"][0],config.GetAllocator()),config.GetAllocator());
                break;
            case 13: config.AddMember(rapidjson::StringRef(kCsvLedgerSegmentsField),Value(plans,config.GetAllocator()),config.GetAllocator()); break;
        }
        if(fault==1)config.RemoveMember(kCsvLedgerSegmentsField);
        f.Replace("configuration.json",config); f.Manifest();
        if(fault==0||fault==1||fault==14||fault==15) {
            auto manifest=f.Read("manifest.json");
            if(fault==0)manifest.RemoveMember(kCsvLedgerSegmentsField);
            if(fault==1) {
                manifest["schema"].SetString("robo_dyna.guided_plate_artifacts.v2",manifest.GetAllocator());
                manifest.AddMember(rapidjson::StringRef(kCsvLedgerSegmentsField),
                                  Value(saved[kCsvLedgerSegmentsField],manifest.GetAllocator()),manifest.GetAllocator());
            }
            if(fault==14)manifest[kCsvLedgerSegmentsField][0]["header_sha256"].SetString(std::string(64,'b').c_str(),manifest.GetAllocator());
            if(fault==15)manifest["schema"].SetString("robo_dyna.guided_plate_artifacts.v1",manifest.GetAllocator());
            f.Replace("manifest.json",manifest);
        }
        SCOPED_TRACE(fault); ExpectRejectedPreservingReplay(replay,f.directory);
        f.Replace("configuration.json",saved); f.Manifest();
    }
    EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
}

TEST(AcceptedReplayGuided, SegmentedActualLateRowsAreCheckedBeyondRehashedInventory) {
    GuidedBundle f; f.Segmented(); AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok); ASSERT_EQ(replay.Load(2).status,ReplayStatus::Ok);
    const auto config=f.Read("configuration.json"); const auto plans=ParseCsvLedgerSegments(config[kCsvLedgerSegmentsField]);
    const auto file=plans[2].segments.back().file;
    const auto saved=ReadBounded(f.directory/file,kArtifactFileCap);
    const auto start=saved.rfind('\n',saved.size()-2)+1;
    const auto original_line=saved.substr(start,saved.size()-start-1);
    for(unsigned fault=0;fault<16;++fault) {
        std::array<std::string,7> fields; std::istringstream input(original_line);
        for(auto& value:fields)ASSERT_TRUE(bool(std::getline(input,value,',')));
        switch(fault) {
            case 0: fields[0]="8"; break;
            case 1: fields[1]="20998"; break;
            case 2: fields[2]="0"; break;
            case 3: fields[2]="21000"; break;
            case 4: fields[4]="20999"; break;
            case 5: fields[3]="nan"; break;
            case 6: fields[5]="-1"; break;
            case 7: fields[6]="inf"; break;
            case 13: fields[6]=std::string(kCsvLedgerRowCap,'1'); break;
            case 14: fields[4]="21000x"; break;
        }
        std::ostringstream row;
        for(unsigned i=0;i<(fault==8?6u:7u);++i) { if(i)row<<','; row<<fields[i]; }
        if(fault==9)row<<",2";
        row<<'\n';
        auto bytes=saved.substr(0,start)+row.str();
        if(fault==10)bytes.pop_back();
        if(fault==11)bytes+=original_line+'\n';
        if(fault==12)bytes.replace(0,8,"owner_jd");
        if(fault==15)bytes=saved.substr(0,start);
        f.Replace(file,bytes); f.Manifest(); // Fresh hashes: row validation must find these defects.
        SCOPED_TRACE(fault); ExpectRejectedPreservingReplay(replay,f.directory);
        f.Replace(file,saved); f.Manifest();
    }
    EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
}

TEST(AcceptedReplayGuided, SegmentedSavedEndpointAttemptsBindTheirActualPrecedingRows) {
    GuidedBundle f; f.Segmented(); AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok); ASSERT_EQ(replay.Load(2).status,ReplayStatus::Ok);
    for(const auto& item:std::array<std::pair<const char*,std::uint64_t>,2>{{
            {"accepted-010500.fields.json",10502},{"accepted-021000.fields.json",21001}}}) {
        const auto saved=f.Read(item.first); auto changed=f.Read(item.first);
        changed["element_evaluation_attempt"].SetUint64(item.second);
        changed["contact_evaluation_attempt"].SetUint64(item.second);
        for(auto& parent:changed["contact_parents"].GetArray())parent["attempt"].SetUint64(item.second);
        f.Replace(item.first,changed); f.Manifest(); ExpectRejectedPreservingReplay(replay,f.directory);
        f.Replace(item.first,saved); f.Manifest();
    }
    EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
}

TEST(AcceptedReplayGuided, MatchingCorruptContributorStampsDoNotReplaceOwnerOrClockChecks) {
    GuidedBundle f; f.Segmented(); AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok); ASSERT_EQ(replay.Load(2).status,ReplayStatus::Ok);
    const auto config=f.Read("configuration.json"); const auto plans=ParseCsvLedgerSegments(config[kCsvLedgerSegmentsField]);
    std::array<std::string,3> saved;
    for(unsigned i=0;i<3;++i)saved[i]=ReadBounded(f.directory/plans[i].segments.back().file,kArtifactFileCap);
    for(unsigned fault=0;fault<5;++fault) {
        for(unsigned ledger=0;ledger<3;++ledger) {
            const auto start=saved[ledger].rfind('\n',saved[ledger].size()-2)+1;
            std::istringstream input(saved[ledger].substr(start)); std::array<std::string,7> fields;
            for(auto& value:fields)ASSERT_TRUE(bool(std::getline(input,value,',')));
            if(fault==0)fields[0]="8";
            if(fault==1)fields[1]="20998";
            if(fault==2)fields[2]="21000";
            if(fault==3)fields[3]="0.5";
            if(fault==4)fields[5]="2";
            std::ostringstream row;
            for(unsigned i=0;i<6;++i) { if(i)row<<','; row<<fields[i]; }
            row<<','<<ledger+1<<'\n';
            f.Replace(plans[ledger].segments.back().file,saved[ledger].substr(0,start)+row.str());
        }
        f.Manifest(); SCOPED_TRACE(fault); ExpectRejectedPreservingReplay(replay,f.directory);
        for(unsigned ledger=0;ledger<3;++ledger)f.Replace(plans[ledger].segments.back().file,saved[ledger]);
        f.Manifest();
    }
    EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
}

TEST(AcceptedReplayGuided, AuthenticatedHeaderStillRequiresTheFixedIdentityColumns) {
    GuidedBundle f; f.Segmented(); AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
    const auto original=f.Read("configuration.json"); auto config=f.Read("configuration.json");
    const auto plans=ParseCsvLedgerSegments(config[kCsvLedgerSegmentsField]);
    std::vector<std::string> saved;
    std::string header;
    for(const auto& segment:plans[2].segments) {
        saved.push_back(ReadBounded(f.directory/segment.file,kArtifactFileCap));
        auto altered=saved.back(); altered.replace(0,8,"owner_jd");
        header=altered.substr(0,altered.find('\n')+1); f.Replace(segment.file,altered);
    }
    // Same-length, valid generic header, rehashed consistently in both plans:
    // only the guided identity-column contract must reject this alteration.
    config[kCsvLedgerSegmentsField][2]["header_sha256"].SetString(Sha256(header).c_str(),config.GetAllocator());
    f.Replace("configuration.json",config); f.Manifest(); ExpectRejectedPreservingReplay(replay,f.directory);
    for(unsigned i=0;i<saved.size();++i)f.Replace(plans[2].segments[i].file,saved[i]);
    f.Replace("configuration.json",original); f.Manifest();
    EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
}

TEST(AcceptedReplayGuided, SegmentedMissingExtraFilesAndFalseV2ScopeReject) {
    GuidedBundle f; f.Segmented(); AcceptedReplay replay;
    ASSERT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
    const auto config=f.Read("configuration.json"); const auto plans=ParseCsvLedgerSegments(config[kCsvLedgerSegmentsField]);
    const auto file=plans[2].segments.back().file;
    const auto saved=ReadBounded(f.directory/file,kArtifactFileCap);
    fs::remove(f.directory/file); f.Manifest(); ExpectRejectedPreservingReplay(replay,f.directory);
    WriteBytes(f.directory/file,saved); f.Manifest();
    WriteBytes(f.directory/"contact-intervals-0002.csv","undeclared segment\n"); f.Manifest();
    ExpectRejectedPreservingReplay(replay,f.directory);
    fs::remove(f.directory/"contact-intervals-0002.csv"); f.Manifest();
    EXPECT_EQ(replay.Open(f.directory).status,ReplayStatus::Ok);
    GuidedBundle unsplit; unsplit.Segmented(10); // Valid metadata and rows, but no actual split.
    ExpectRejectedPreservingReplay(replay,unsplit.directory);
}

TEST(AcceptedReplayGuided, ExplicitRetainedGuidedBundleUsesTheSameStrictValidation) {
    const char* path=std::getenv("ROBO_DYNA_GUIDED_REPLAY_FIXTURE");
    if(!path || !*path) GTEST_SKIP()<<"Explicit retained guided bundle not supplied";
    AcceptedReplay replay; const auto open=replay.Open(path); ASSERT_EQ(open.status,ReplayStatus::Ok)<<open.diagnostic;
    ASSERT_EQ(replay.info()->kind,ReplayKind::GuidedPlate); ASSERT_TRUE(replay.wall());
    for(std::size_t i=0;i<replay.info()->frame_count;++i) {
        const auto report=replay.Load(i); ASSERT_EQ(report.status,ReplayStatus::Ok)<<report.diagnostic;
    }
    EXPECT_EQ(replay.frame()->epoch,replay.info()->final_epoch);
    EXPECT_EQ(Bits(replay.frame()->time),Bits(replay.info()->final_time));
}
} // namespace
} // namespace crash::output
