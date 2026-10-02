// Real inherited MPI transport and rank-topology verification; no simulated physics.
#include "GeometryFixture.h"
#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

#include <iostream>
#include <stdexcept>

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

// The test exposes protected transport utilities without replacing MPI or adding
// a second mechanics owner. Virtual mechanics operations are deliberately unused.
class TransportProbe final : public chrono::vehicle::ChVehicleCosimBaseNode {
  public:
    explicit TransportProbe(NodeType type) : ChVehicleCosimBaseNode("transport-probe"), type_(type) {
        SetVerbose(false);
    }
    NodeType GetNodeType() const override { return type_; }
    void Synchronize(int, double) override { throw std::logic_error("No physics in transport test"); }
    void Advance(double) override { throw std::logic_error("No physics in transport test"); }
    void OutputData(int) override {}
    void OutputVisualizationData(int) override {}
    using ChVehicleCosimBaseNode::RecvGeometry;
    using ChVehicleCosimBaseNode::SendGeometry;
    bool HasExpectedNodeCounts() const {
        return m_num_wheeled_mbs_nodes == 1 && m_num_tracked_mbs_nodes == 0 &&
               m_num_terrain_nodes == 2 && m_num_tire_nodes == 1;
    }

  private:
    chrono::ChSystem* GetSystemPostprocess() const override { return nullptr; }
    NodeType type_;
};
}  // namespace

int main(int argc, char** argv) {
    if (MPI_Init(&argc, &argv) != MPI_SUCCESS)
        return 2;
    int rank = -1;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    try {
        using namespace chrono::vehicle;
        int size = 0;
        MPI_Comm_size(MPI_COMM_WORLD, &size);
        Require(size == 4, "Transport coupon requires exactly four local ranks");
        Require(cosim::InitializeFramework(3) == MPI_ERR_OTHER, "Insufficient-rank admission did not reject");
        Require(!cosim::IsFrameworkInitialized(), "Rejected setup published a terrain communicator");
        Require(cosim::InitializeFramework(1) == MPI_SUCCESS, "Real framework setup failed");
        const bool terrain = rank == 1 || rank == 3;
        Require(cosim::IsFrameworkInitialized() == terrain, "Terrain communicator membership changed");
        if (terrain) {
            int terrain_size = 0, terrain_rank = -1, sum = 0;
            auto communicator = cosim::GetTerrainIntracommunicator();
            MPI_Comm_size(communicator, &terrain_size);
            MPI_Comm_rank(communicator, &terrain_rank);
            MPI_Allreduce(&rank, &sum, 1, MPI_INT, MPI_SUM, communicator);
            Require(terrain_size == 2 && sum == 4 && terrain_rank == (rank == 1 ? 0 : 1),
                    "Terrain communicator rank ordering or real collective failed");
        }
        auto type = rank == 0 ? TransportProbe::NodeType::MBS_WHEELED :
                    rank == 2 ? TransportProbe::NodeType::TIRE : TransportProbe::NodeType::TERRAIN;
        TransportProbe node(type);
        node.Initialize();
        Require(node.HasExpectedNodeCounts(), "Original all-rank node census changed");
        Require(node.GetStepSize() == 1e-4, "Original default integration step changed");
        chrono::utils::ChBodyGeometry geometry;
        if (rank == 0)
            node.SendGeometry(geometry, TERRAIN_NODE_RANK);
        if (rank == 1) {
            node.RecvGeometry(geometry, MBS_NODE_RANK);
            Require(geometry.materials.empty() && geometry.coll_meshes.empty(), "Empty message was not empty");
        }
        MPI_Barrier(MPI_COMM_WORLD);
        if (rank == 2) {
            geometry = robodyna::verification::MakeTransportGeometry();
            node.SendGeometry(geometry, TERRAIN_NODE_RANK);
        }
        if (rank == 1) {
            node.RecvGeometry(geometry, TIRE_NODE_RANK(0));
            robodyna::verification::CheckTransportGeometry(geometry);
        }
        MPI_Barrier(MPI_COMM_WORLD);
        if (rank == 0)
            std::cout << "{\"schema\":\"robodyna.vehicle_cosim_transport.v1\",\"status\":\"passed\","
                         "\"ranks\":4,\"terrain_ranks\":2,\"messages\":2,\"mesh_triangles\":2,"
                         "\"scope\":\"framework-and-original-geometry-transport-only\"}\n";
        MPI_Finalize();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Rank " << rank << ": " << error.what() << '\n';
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }
}
