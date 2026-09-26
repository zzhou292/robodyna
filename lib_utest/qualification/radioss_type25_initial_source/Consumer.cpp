#include "lib_src/collision/RadiossType25InitialState.h"
#include <type_traits>
int main() {
  namespace s=tlfea::contact::radioss_type25::initial_source;
  static_assert(!std::is_copy_constructible_v<s::PreparedSource>);
  static_assert(!std::is_copy_constructible_v<s::DeviceSeed>);
  s::PreparedSource prepared;s::DeviceSeed seed;
  return prepared.prepared()||seed.prepared()||prepared.removals().by_secondary.entry_count;
}
