#include "AcceptedReplay.h"
#include "ArtifactIO.h"
#include "MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace crash::output {
namespace {
namespace fs = std::filesystem;

class BundleFixture {
  public:
    fs::path directory;
    bool coupon;
    explicit BundleFixture(bool elastic = false) : coupon(elastic) {
        auto pattern = (fs::temp_directory_path()/"accepted-replay-XXXXXX").string();
        std::vector<char> name(pattern.begin(), pattern.end()); name.push_back(0);
        const auto created = ::mkdtemp(name.data());
        Require(created, "Could not create replay test directory"); directory = created;
        chrono::ChTriangleMeshConnected mesh;
        mesh.GetCoordsVertices() = {{.1,.05,0},{0,.05,0},{0,-.05,0},{.1,-.05,0},{.2,.05,0},{.2,-.05,0}};
        mesh.GetIndicesVertices() = {{0,1,2},{0,2,3},{4,0,3},{4,3,5}};
        Document configuration; configuration.SetObject();
        String(configuration, "schema", coupon ? "robo_dyna.elastic_coupon_configuration.v1" : "tlfea.normal_impact_configuration.v1");
        Number(configuration,coupon ? "fixed_dt_s" : "dt_s",.1);
        Number(configuration,coupon ? "half_period_horizon_s" : "requested_horizon_s",1);
        if (coupon) {
            Integer(configuration,"owner_id",7); Integer(configuration,"run_id",1); Integer(configuration,"topology_id",42);
            Integer(configuration,"required_steps",10);
            Value vertices(rapidjson::kArrayType), faces(rapidjson::kArrayType);
            for (unsigned n = 0; n < 6; ++n) {
                Value row(rapidjson::kArrayType);
                for (unsigned value : {n,1u,1u,n+1}) row.PushBack(value,configuration.GetAllocator());
                vertices.PushBack(row,configuration.GetAllocator());
            }
            for (const auto& face : mesh.GetIndicesVertices()) {
                Value row(rapidjson::kArrayType);
                for (int value : {face[0],face[1],face[2],1,1,1,1,0,0}) row.PushBack(value,configuration.GetAllocator());
                faces.PushBack(row,configuration.GetAllocator());
            }
            configuration.AddMember("vertex_binding",vertices,configuration.GetAllocator());
            configuration.AddMember("triangle_binding",faces,configuration.GetAllocator());
        } else WriteMeshFiles(directory,"canonical-wall",mesh);
        WriteJson(directory/"configuration.json",configuration);
        for (unsigned frame = 0; frame < 3; ++frame) {
            for (auto& position : mesh.GetCoordsVertices()) position.z() = frame*.01;
            const auto stem = Stem(frame);
            WriteMeshFiles(directory,stem,mesh);
            if (coupon) {
                Document fields; fields.SetObject(); String(fields,"schema","robo_dyna.elastic_coupon_fields.v1");
                Integer(fields,"owner_id",7); Integer(fields,"accepted_epoch",5*frame); Number(fields,"accepted_time_s",.5*frame);
                Value positions(rapidjson::kArrayType);
                for (const auto& position : mesh.GetCoordsVertices())
                    for (unsigned axis = 0; axis < 3; ++axis) positions.PushBack(position[axis],fields.GetAllocator());
                fields.AddMember("position_xyz_m",positions,fields.GetAllocator());
                WriteJson(directory/(stem+".fields.json"),fields);
            }
        }
        WriteBytes(directory/"accepted-frames.csv", Rows());
        WriteBytes(directory/"accepted-intervals.csv","fixture_interval_record\n");
        Document final; final.SetObject(); Integer(final,"accepted_epoch",10); Number(final,"accepted_time_s",1);
        Integer(final,"owner_id",7); Integer(final,"node_count",6); Integer(final,"saved_frames",3);
        WriteJson(directory/"final-metrics.json",final); Manifest();
    }
    ~BundleFixture() { std::error_code error; fs::remove_all(directory,error); }
    static std::string Stem(unsigned frame) { return frame == 0 ? "accepted-000000" : frame == 1 ? "accepted-000005" : "accepted-000010"; }
    static std::string Rows() {
        return "owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n"
            "7,0,0,accepted-000000.mesh.json,accepted-000000.obj\n"
            "7,5,0.5,accepted-000005.mesh.json,accepted-000005.obj\n"
            "7,10,1,accepted-000010.mesh.json,accepted-000010.obj\n";
    }
    Document ReadJson(const std::string& name) const {
        const auto bytes = ReadBounded(directory/name,32*1024*1024);
        Document doc; doc.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
        Require(!doc.HasParseError(),"Invalid test JSON"); return doc;
    }
    void Replace(const std::string& name, const std::string& bytes) {
        fs::remove(directory/name); WriteBytes(directory/name,bytes);
    }
    void Replace(const std::string& name, const Document& doc) {
        fs::remove(directory/name); WriteJson(directory/name,doc);
    }
    void Manifest() {
        fs::remove(directory/"manifest.json");
        Document manifest; manifest.SetObject();
        String(manifest,"schema",coupon ? "robo_dyna.elastic_coupon_artifacts.v1" : "tlfea.normal_impact_artifacts.v1");
        String(manifest,"status","completed"); Boolean(manifest,"shell_model",coupon); Boolean(manifest,"vehicle_model",false);
        Integer(manifest,"accepted_epoch",10); Number(manifest,"accepted_time_s",1);
        Value inventory(rapidjson::kArrayType);
        for (const auto& file : fs::directory_iterator(directory)) {
            const auto bytes = ReadBounded(file.path(),32*1024*1024), hash = Sha256(bytes);
            const auto name = file.path().filename().string();
            Value item(rapidjson::kObjectType), path(name.c_str(),manifest.GetAllocator()), digest(hash.c_str(),manifest.GetAllocator());
            item.AddMember("file",path,manifest.GetAllocator()); item.AddMember("sha256",digest,manifest.GetAllocator());
            item.AddMember("bytes",Value().SetUint64(bytes.size()),manifest.GetAllocator()); inventory.PushBack(item,manifest.GetAllocator());
        }
        manifest.AddMember("artifacts",inventory,manifest.GetAllocator()); WriteJson(directory/"manifest.json",manifest);
    }
};

TEST(AcceptedReplay, BothSchemasStreamExactFramesAndOnlyArchivedIdentity) {
    for (bool coupon : {false,true}) {
        BundleFixture fixture(coupon); AcceptedReplay replay;
        const auto report = replay.Open(fixture.directory);
        ASSERT_EQ(report.status,ReplayStatus::Ok) << report.diagnostic;
        ASSERT_NE(replay.info(),nullptr); EXPECT_EQ(replay.info()->owner_id,7u);
        EXPECT_EQ(replay.info()->frame_count,3u); EXPECT_EQ(replay.info()->node_count,6u);
        EXPECT_EQ(replay.info()->triangle_count,4u); EXPECT_EQ(bool(replay.wall()),!coupon);
        EXPECT_EQ(replay.info()->run_id,coupon?1u:0u); EXPECT_EQ(replay.info()->topology_id,coupon?42u:0u);
        EXPECT_DOUBLE_EQ(replay.info()->bounds_min[2],0); EXPECT_DOUBLE_EQ(replay.info()->bounds_max[2],.02);
        for (unsigned frame : {2u,1u,0u}) {
            ASSERT_EQ(replay.Load(frame).status,ReplayStatus::Ok);
            EXPECT_EQ(replay.frame()->index,frame); EXPECT_EQ(replay.frame()->epoch,5*frame);
            EXPECT_EQ(Bits(replay.frame()->time),Bits(.5*frame));
            EXPECT_EQ(Bits(replay.frame()->mesh->GetCoordsVertices()[0].z()),Bits(frame*.01));
        }
    }
}

TEST(AcceptedReplay, MissingIncompleteAndWrongSchemaManifestsAreRejected) {
    for (unsigned fault = 0; fault < 5; ++fault) {
        BundleFixture fixture; auto manifest = fixture.ReadJson("manifest.json");
        if (fault == 0) fs::remove(fixture.directory/"manifest.json");
        if (fault == 1) { manifest["status"].SetString("failed",manifest.GetAllocator()); fixture.Replace("manifest.json",manifest); }
        if (fault == 2) WriteBytes(fixture.directory/"failure.json","{}");
        if (fault == 3) { manifest["schema"].SetString("unknown",manifest.GetAllocator()); fixture.Replace("manifest.json",manifest); }
        if (fault == 4) fixture.Replace("manifest.json","{");
        AcceptedReplay replay; EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
        EXPECT_EQ(replay.frame(),nullptr);
    }
}

TEST(AcceptedReplay, InventoryRejectsDuplicateUnsafeHashSizeAndTotalCap) {
    for (unsigned fault = 0; fault < 6; ++fault) {
        BundleFixture fixture; auto manifest = fixture.ReadJson("manifest.json");
        auto& entries = manifest["artifacts"]; auto& first = entries[0];
        if (fault == 0) { Value copy(first,manifest.GetAllocator()); entries.PushBack(copy,manifest.GetAllocator()); }
        if (fault == 1) first["file"].SetString("../outside",manifest.GetAllocator());
        if (fault == 2) first["sha256"].SetString(std::string(64,'0').c_str(),manifest.GetAllocator());
        if (fault == 3) first["bytes"].SetUint64(first["bytes"].GetUint64()+1);
        if (fault == 4) first["bytes"].SetUint64(32*1024*1024+1);
        if (fault == 5) for (auto& entry : entries.GetArray()) entry["bytes"].SetUint64(32*1024*1024);
        fixture.Replace("manifest.json",manifest); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    }
}

TEST(AcceptedReplay, RequiredWallAndConfiguredTimeCannotBeOmittedOrRelabeled) {
    for (unsigned fault = 0; fault < 4; ++fault) {
        BundleFixture fixture;
        if (fault == 0) fs::remove(fixture.directory/"canonical-wall.mesh.json");
        else {
            auto configuration = fixture.ReadJson("configuration.json");
            if (fault == 1) configuration["dt_s"].SetDouble(0);
            if (fault == 2) configuration["dt_s"].SetDouble(.11);
            if (fault == 3) configuration["requested_horizon_s"].SetDouble(1.1);
            fixture.Replace("configuration.json",configuration);
        }
        fixture.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    }
}

TEST(AcceptedReplay, OwnerInitialFinalAndIntermediateOrderingMustMatch) {
    for (unsigned fault = 0; fault < 6; ++fault) {
        BundleFixture fixture; auto rows = BundleFixture::Rows();
        if (fault == 0) rows.replace(rows.find("7,5,0.5"),7,"8,5,0.5");
        if (fault == 1) rows.replace(rows.find("7,0,0,"),6,"7,0,0.1,");
        if (fault == 2) rows.replace(rows.find("7,10,1,"),7,"7,10,1.1,");
        if (fault == 3) rows.replace(rows.find("7,5,0.5"),7,"7,5,2.0");
        if (fault == 4) rows.erase(rows.find("7,10,1,"));
        if (fault == 5) rows.insert(rows.find("7,10,1,"),"7,5,0.5,accepted-000005.mesh.json,accepted-000005.obj\n");
        fixture.Replace("accepted-frames.csv",rows); fixture.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    }
}

TEST(AcceptedReplay, HashValidLateMeshRejectsConnectivityCountsAndDegenerateGeometry) {
    for (unsigned fault = 0; fault < 5; ++fault) {
        BundleFixture fixture; const auto name = BundleFixture::Stem(2)+".mesh.json";
        auto document = fixture.ReadJson(name); auto& mesh = document["mesh"];
        if (fault == 0) mesh["m_face_v_indices"][0]["x"].SetInt(99);
        if (fault == 1) mesh["m_face_v_indices"][0]["x"].SetInt(3);
        if (fault == 2) mesh["m_face_v_indices"].PopBack();
        if (fault == 3) mesh["m_vertices"].PushBack(Value(mesh["m_vertices"][0],document.GetAllocator()),document.GetAllocator());
        if (fault == 4) mesh["m_face_v_indices"][0]["x"].SetInt(1);
        fixture.Replace(name,document); fixture.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    }
}

TEST(AcceptedReplay, NonfiniteAndOverCapMeshAreRejectedBeforeChronoAllocation) {
    for (bool overflow : {false,true}) {
        BundleFixture fixture; const auto name = BundleFixture::Stem(2)+".mesh.json";
        if (overflow) {
            auto document = fixture.ReadJson(name); auto& vertices = document["mesh"]["m_vertices"];
            while (vertices.Size() <= 4096) vertices.PushBack(Value(vertices[0],document.GetAllocator()),document.GetAllocator());
            fixture.Replace(name,document);
        } else {
            auto bytes = ReadBounded(fixture.directory/name,32*1024*1024);
            const auto x = bytes.find("\"x\""); const auto begin = bytes.find(':',x)+1, end = bytes.find(',',begin);
            bytes.replace(begin,end-begin," 1e999"); fixture.Replace(name,bytes);
        }
        fixture.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    }
}

TEST(AcceptedReplay, CouponFieldsAndConfigurationMustBindActualOwnerAndGeometry) {
    for (unsigned fault = 0; fault < 4; ++fault) {
        BundleFixture fixture(true);
        const auto name = fault == 0 ? "configuration.json" : BundleFixture::Stem(2)+".fields.json";
        auto document = fixture.ReadJson(name);
        if (fault < 2) document["owner_id"].SetUint64(8);
        if (fault == 2) document["accepted_time_s"].SetDouble(.9);
        if (fault == 3) document["position_xyz_m"][0].SetDouble(.3);
        fixture.Replace(name,document); fixture.Manifest(); AcceptedReplay replay;
        EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    }
}

TEST(AcceptedReplay, FailedLoadOrReopenPreservesCurrentVerifiedFrame) {
    BundleFixture fixture; AcceptedReplay replay;
    ASSERT_EQ(replay.Open(fixture.directory).status,ReplayStatus::Ok);
    ASSERT_EQ(replay.Load(1).status,ReplayStatus::Ok);
    const auto saved = *replay.frame(); const auto info = *replay.info();
    fixture.Replace(BundleFixture::Stem(2)+".mesh.json","corrupted");
    EXPECT_EQ(replay.Load(2).status,ReplayStatus::InvalidFrame);
    EXPECT_EQ(replay.Load(1000).status,ReplayStatus::InvalidFrame);
    EXPECT_EQ(replay.Open(fixture.directory).status,ReplayStatus::InvalidBundle);
    EXPECT_EQ(replay.frame()->mesh,saved.mesh); EXPECT_EQ(replay.frame()->index,saved.index);
    EXPECT_EQ(replay.frame()->epoch,saved.epoch); EXPECT_EQ(replay.frame()->time,saved.time);
    EXPECT_EQ(replay.info()->owner_id,info.owner_id); EXPECT_EQ(replay.info()->bounds_max,info.bounds_max);
}

TEST(AcceptedReplay, ExplicitRetainedBundlePassesTheSameValidation) {
    const auto directory = std::getenv("ROBO_DYNA_REPLAY_FIXTURE");
    if (!directory) GTEST_SKIP() << "Set ROBO_DYNA_REPLAY_FIXTURE to qualify a retained result bundle";
    AcceptedReplay replay; const auto report = replay.Open(directory);
    ASSERT_EQ(report.status,ReplayStatus::Ok) << report.diagnostic;
    ASSERT_GT(replay.info()->frame_count,1u);
    ASSERT_EQ(replay.Load(replay.info()->frame_count-1).status,ReplayStatus::Ok);
    EXPECT_EQ(replay.frame()->epoch,replay.info()->final_epoch);
    EXPECT_EQ(Bits(replay.frame()->time),Bits(replay.info()->final_time));
}
}  // namespace
}  // namespace crash::output
