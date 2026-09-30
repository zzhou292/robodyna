#include "output/recovered_frames/Replay.h"
#include "output/physical_run/ViewerInput.h"
#include <iostream>

int main(int argc,char** argv) {
    namespace out=crash::output;
    try {
        out::Require(argc==5,"Usage: recover-physical-samples INTERRUPTED_ARCHIVE EMPTY_DESTINATION SOURCE_VIEWER_RECEIPT STOP_REASON");
        std::filesystem::path receipt=std::filesystem::absolute(argv[3]);
        const auto bytes=out::ReadBounded(receipt,out::physical_run::ViewerInputByteCap);
        const out::full_shell::RecordFile file{receipt.filename().string(),out::Sha256(bytes),bytes.size()};
        const auto authority=out::physical_run::ReadViewerInput(receipt.parent_path(),file);
        const auto recovered=out::recovered_frames::Recover(argv[1],argv[2],authority.source,
            authority.mapping_sha256,argv[4]);
        const auto replay=out::recovered_frames::Replay::Open(argv[2],recovered);
        const auto& last=replay.frames().back().stamp;
        std::cout<<"Recovered recorded samples: "<<replay.frames().size()<<"; last epoch "<<last.epoch
            <<"; interval ledger unavailable; requested horizon not certified\n"
            <<recovered.file<<' '<<recovered.bytes<<' '<<recovered.sha256<<'\n';
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"Recovery refused: "<<error.what()<<'\n';return 1;
    }
}
