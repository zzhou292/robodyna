// Exercise the original MPI communicator and wire schema, without a mechanics model.
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

#include <iostream>
#include <stdexcept>

namespace {
void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
    using namespace chrono::synchrono;
    int rank = -1;
    {
        // The retained communicator owns MPI_Init and MPI_Finalize itself.
        SynMPICommunicator communicator(argc, argv);
        rank = communicator.GetRank();
        try {
            Require(communicator.GetNumRanks() == 2, "This coupon requires exactly two local ranks");
            communicator.Initialize();
            for (int round = 0; round < 3; ++round) {
                const int sent = round == 2 ? 0 : (round + 1) * (rank + 1);
                SynMessageList outgoing;
                for (int i = 0; i < sent; ++i)
                    outgoing.push_back(std::make_shared<SynSimulationMessage>(
                        AgentKey(rank, 100 * round + i), AgentKey(1 - rank, 200 + i), (i % 2) == 0));
                communicator.AddOutgoingMessages(outgoing);
                communicator.Synchronize();
                const auto& received = communicator.GetMessages();
                const int expected = round == 2 ? 0 : (round + 1) * (2 - rank);
                Require(received.size() == static_cast<size_t>(expected), "Gathered message count differs");
                for (int i = 0; i < expected; ++i) {
                    auto message = std::dynamic_pointer_cast<SynSimulationMessage>(received[i]);
                    Require(message != nullptr, "Original message factory produced the wrong type");
                    Require(message->GetSourceKey().GetNodeID() == 1 - rank &&
                                message->GetSourceKey().GetAgentID() == 100 * round + i &&
                                message->GetDestinationKey().GetNodeID() == rank &&
                                message->GetDestinationKey().GetAgentID() == 200 + i &&
                                message->m_quit_sim == ((i % 2) == 0),
                            "Cross-rank wire fields changed");
                }
                communicator.Reset();
                communicator.Barrier();
            }
        } catch (const std::exception& error) {
            std::cerr << "Rank " << rank << ": " << error.what() << '\n';
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    int finalized = 0;
    MPI_Finalized(&finalized);
    if (!finalized)
        return 2;
    if (rank == 0)
        std::cout << "{\"schema\":\"robodyna.distributed_mpi_test.v1\",\"status\":\"passed\","
                     "\"ranks\":2,\"rounds\":3,\"scope\":\"original-communicator-and-wire-schema\"}\n";
    return 0;
}
