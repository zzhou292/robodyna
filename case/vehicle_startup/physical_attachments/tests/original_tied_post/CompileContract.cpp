#include <cstdlib>
#include <type_traits>
#include "../../OriginalTiedPost.h"
#include "../PreparePost.h"
namespace crash::cases::vehicle_startup::physical_attachments::test {
using Search = TiedSearchFinalized (*)(const modelio::tied_shell::TiedShellDeclaration&, const std::string&);
using Post = TiedSearchPostKinChk (*)(const TiedSearchFinalized&, const modelio::tied_shell::TiedClassificationContext&);
static_assert(std::is_same_v<decltype(&FinalizeOriginalTiedSearch),Search>);
static_assert(std::is_same_v<decltype(&PrepareOriginalTiedPost),Post>);
using Fixture = TiedSearchPostKinChk (*)(const modelio::physical_scope::PhysicalScope&, const std::string&);
static_assert(std::is_same_v<decltype(&PreparePost),Fixture>);
} // namespace crash::cases::vehicle_startup::physical_attachments::test
