#include "lib_src/elements/qbat/QbatBatch.h"
#include "lib_src/elements/qbat/QbatBatchAdvance.h"
int main() {
  namespace qb=tl::fea::qbat;
  qb::Batch owner;
  qb::BatchDiagnostics diagnostics;
  if(owner.CopyAcceptedDiagnostics({},&diagnostics).status!=qb::BatchStatus::NotInitialized)return 1;
  qb::ForceTrial force;
  if(qb::EvaluateForce({}, {}, {}, {}, {}, force)!=qb::Status::kInvalidReference)return 2;
  qb::batch_detail::Element element;
  qb::BatchResult accepted,trial;
  if(qb::batch_detail::AdvanceIntoTrial(element,accepted,{},trial)!=qb::Status::kInvalidInput)return 3;
  return 0;
}
