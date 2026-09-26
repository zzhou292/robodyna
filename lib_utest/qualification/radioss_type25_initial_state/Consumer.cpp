#include "lib_src/collision/RadiossType25InitialStateValues.h"
int main() {
  namespace i=tlfea::contact::radioss_type25::initial_state;
  i::PairInput input;i::PairResult output;output.local_main=17;
  if(i::EvaluatePair(input,&output)!=i::Status::InvalidInput||output.local_main!=17)return 1;
  i::RowInput rows;rows.profile.partitions=2;i::Winner winner;
  if(i::ReduceRow(rows,0,&winner).status!=i::Status::UnsupportedProfile)return 2;
  return 0;
}
