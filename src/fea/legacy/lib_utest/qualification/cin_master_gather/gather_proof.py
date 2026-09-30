"""Checked gather-only reversals and exact scheduling/arithmetic source proofs."""
from pathlib import Path
import hashlib
import re


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASELINE = {
    'lib_src/solvers/BUILD.bazel': (9102, '30f992667b5c99ea46e463c90e307bafd0ebd97d166daae83959fc75c4b16fd3'),
    'lib_src/solvers/CMakeLists.txt': (1890, 'd51ac417e6deb7222b222d5e2ca2b7d60593a022e9355bedf488da815d83ba2c'),
    'lib_src/solvers/ExplicitNodalCinStep.cu': (10596, '8ca0b97924e0fe6c70d05389acb34c13f068c5b7a88ed5e743aec9731b8ecf97'),
    'lib_src/solvers/FENodalState.cu': (29691, '6e5ec0253f5e7de82cb8f97ddcc2915462eb2e8302c74f4b12c046be0cbd9c8a'),
    'lib_src/solvers/NodalAssemblyCinForecast.cpp': (3977, '4e2e243008fb44903f9fea74d08e8e9026a5423e17bd9b48d09b5294a76454fc'),
    'lib_src/solvers/NodalCinLayout.h': (4862, '2bef5df455d4ac97ec47f23d492dfb271c3a13eb2686a6480114e0abd19cc8b2'),
    'lib_src/solvers/NodalCinStartup.cpp': (12277, '992c57f259ead45540c4934ccec45a9ffdbe073175a13b81bc39943f3d952dff'),
    'lib_src/solvers/NodalCinStorage.cu': (3430, '6b585e9057ca2edd4296a20ed8fd7a84fdd4002e7a1f744936e3d2489ce815ba'),
    'lib_src/solvers/NodalCinStorage.h': (1970, 'c2cb8cc77f81e421f49856a8b99ea7c5ada78d728430a65ab1a02c1b823880b7'),
    'lib_src/solvers/cin_advance/ForceTransfers.cu': (911, '9594d52dba733270d06b3894cac8967f765c4900f1827c4064faa519848a093d'),
    'lib_src/solvers/cin_advance/Input.h': (2499, '5fdc2d74da6764b9e87934e48baaa5be8da1ee4b982eff7a87c3f140eb15f9eb'),
}


def compact(value: str) -> str:
    return re.sub(r"\s+", "", value)


def body(text: str, name: str) -> str:
    match = re.search(r"\b" + re.escape(name) + r"\s*\(", text)
    assert match, name
    start = text.index("{", match.start())
    end, depth = start + 1, 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start + 1:end - 1]


def replace_once(text: str, addition: str, prior: str = "") -> str:
    assert text.count(addition) == 1, addition
    return text.replace(addition, prior, 1)


def original(path: str, text: str) -> str:
    expected_bytes, expected_hash = BASELINE[path]
    data = text.encode()
    assert len(data) == expected_bytes, path
    assert hashlib.sha256(data).hexdigest() == expected_hash, path
    return text


def without_gather_owner(text: str) -> str:
    if hashlib.sha256(text.encode()).hexdigest() == BASELINE[
            "lib_src/solvers/ExplicitNodalCinStep.cu"][1]:
        return text
    markers = [
        '#include "cin_advance/ForceGather.h"\n',
        "transfers_gathered ? input.force_gather.summary->report",
        "const bool parallel_gather =",
        "cin->force_gather}, stream);",
    ]
    if markers[0] not in text:
        assert not any(marker in text for marker in markers[1:])
        return text
    text = replace_once(text, '#include "cin_advance/ForceGather.h"\n')
    text = replace_once(text,
        "bool transfers_prepared = false, bool transfers_gathered = false)",
        "bool transfers_prepared = false)")
    text = replace_once(text,
        "transfers_gathered ? input.force_gather.summary->report\n"
        "      : transfers_prepared ?",
        "transfers_prepared ?")
    addition = """  const bool parallel_gather = parallel_transfers && force_gather::Eligible(input.model, input.force_gather);
  if (parallel_gather) {
    error = force_gather::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
"""
    text = replace_once(text, addition)
    text = replace_once(text,
        "(input, parallel_inputs, parallel_screen, parallel_transfers, parallel_gather);",
        "(input, parallel_inputs, parallel_screen, parallel_transfers);")
    text = replace_once(text,
        "cin->recovery_failure, cin->prepared_drift,\n      cin->force_gather}, stream);",
        "cin->recovery_failure, cin->prepared_drift}, stream);")
    return text


def without_gather_fenodal(text: str) -> str:
    if hashlib.sha256(text.encode()).hexdigest() == BASELINE[
            "lib_src/solvers/FENodalState.cu"][1]:
        return text
    text = replace_once(text, '#include "NodalCinGatherLayout.h"\n')
    return replace_once(text, """    nodal_detail::SelectCinGatherLayout(cin_layout, layout, cin->limits, c.max_device_bytes,
        rigid_layout.host_bytes, sizeof(Impl));
""")


def without_gather_build(text: str) -> str:
    if hashlib.sha256(text.encode()).hexdigest() == BASELINE[
            "lib_src/solvers/BUILD.bazel"][1]:
        return text
    text = replace_once(text, ', "cin_advance/ForceGather.cu"')
    text = replace_once(text,
        '            "NodalCinGatherLayout.h", "cin_advance/ForceGather.h", "cin_advance/ForceGatherTypes.h", "cin_advance/ForceGatherValues.h", "cin_advance/ForceGatherLayout.h", "cin_advance/ForceGatherIncidence.h",\n')
    return replace_once(text, ', "//lib_utils:ordered_node_incidence"')


def restore(path: str, text: str) -> str:
    digest = hashlib.sha256(text.encode()).hexdigest()
    if digest == BASELINE[path][1]:
        return original(path, text)
    if path == "lib_src/solvers/ExplicitNodalCinStep.cu":
        text = without_gather_owner(text)
    elif path == "lib_src/solvers/FENodalState.cu":
        text = without_gather_fenodal(text)
    elif path == "lib_src/solvers/BUILD.bazel":
        text = without_gather_build(text)
    elif path == "lib_src/elements/mapped_shell/BUILD.bazel":
        text = replace_once(text, ', "//lib_utils:ordered_node_incidence"')
        text = replace_once(text, """    visibility = [
        "//lib_src/elements/qbat:__pkg__",
        "//lib_src/elements/qeph:__pkg__",
        "//lib_src/elements/t3:__pkg__",
        "//lib_utest/qualification/cin_master_gather:__pkg__",
    ],
""")
    elif path == "lib_src/elements/mapped_shell/Incidence.h":
        reference = (HERE / "reference/Incidence.h").read_text()
        text = replace_once(text, '#include "lib_utils/OrderedNodeIncidence.h"\n')
        current = """  return util::BuildOrderedNodeIncidence<Slots>(parents, nodes,
      [&](std::size_t parent, unsigned slot) { return elements[parent].nodes[slot]; },
      offsets, offset_count, incidence, incidence_count);
"""
        old_body = body(reference, "BuildIncidence")
        old = old_body[old_body.index(
            "  for (std::size_t node = 0; node <= nodes; ++node)"):]
        text = replace_once(text, current, old)
    elif path == "lib_src/solvers/CMakeLists.txt":
        text = replace_once(text, " cin_advance/ForceGather.cu")
    elif path == "lib_src/solvers/NodalAssemblyCinForecast.cpp":
        text = replace_once(text, '#include "NodalCinGatherLayout.h"\n')
        text = replace_once(text, """  SelectCinGatherLayout(attachment, layout, cin.limits, config.max_device_bytes,
      rigid.host_bytes, sizeof(Impl));
""")
        text = replace_once(text, """  const auto scratch = util::SourceIdentityIndex<16>::Bytes(cin.witness_count) +
      attachment.gather.temporary_bytes;
""", "  const auto scratch = util::SourceIdentityIndex<16>::Bytes(cin.witness_count);\n")
    elif path == "lib_src/solvers/NodalCinLayout.h":
        text = replace_once(text, '#include "cin_advance/ForceGatherLayout.h"\n')
        text = replace_once(text, "  cin_advance::force_gather::Layout gather;\n")
    elif path == "lib_src/solvers/NodalCinStartup.cpp":
        text = replace_once(text, '#include "cin_advance/ForceGatherIncidence.h"\n')
        text = replace_once(text, """  if (layout.gather.device_bytes) {
    const auto& gathered = layout.gather;
    util::HostArena temporary;
    if (!next->gather_host.Initialize(gathered.host_bytes) || !temporary.Initialize(gathered.temporary_bytes))
      return {NodalStatus::ResourceLimit, "CIN master incidence startup allocation failed"};
    auto* nodes = next->gather_host.Construct<std::uint32_t>(gathered.host_nodes);
    auto* offsets = next->gather_host.Construct<std::uint32_t>(gathered.host_offsets);
    auto* incidence = next->gather_host.Construct<std::uint32_t>(gathered.host_incidence);
    auto* dense = temporary.Construct<std::uint32_t>(gathered.dense_offsets);
    const cin::StageView source_view{next->rows.data(), next->dependent.data(),
        std::uint32_t(layout.nodes), std::uint32_t(layout.attachments),
        std::uint32_t(layout.witnesses), next->first_witness.data()};
    std::uint32_t masters = 0;
    if (!cin_advance::force_gather::BuildIncidence(source_view, dense, nodes, offsets,
        incidence, gathered.capacity, masters))
      return {NodalStatus::InvalidInput, "CIN immutable master incidence is inconsistent"};
    next->force_gather.node_count = source_view.node_count;
    next->force_gather.row_count = source_view.row_count;
    next->force_gather.master_count = masters;
    next->force_gather.capacity = std::uint32_t(gathered.capacity);
  }
""")
    elif path == "lib_src/solvers/NodalCinStorage.cu":
        text = replace_once(text, "#include <utility>\n")
        text = replace_once(text, """  if (layout.gather.device_bytes) {
    const auto& gathered = layout.gather;
    force_gather.source_rows = device_rows;
    force_gather.nodes = util::ArenaPointer<std::uint32_t>(arena, gathered.nodes);
    force_gather.offsets = util::ArenaPointer<std::uint32_t>(arena, gathered.offsets);
    force_gather.incidence = util::ArenaPointer<std::uint32_t>(arena, gathered.incidence);
    force_gather.values = util::ArenaPointer<cin_advance::force_gather::Master>(arena, gathered.values);
    force_gather.summary = util::ArenaPointer<cin_advance::force_gather::Summary>(arena, gathered.summary);
    for (const auto pair : {std::pair{gathered.nodes, gathered.host_nodes},
                           std::pair{gathered.offsets, gathered.host_offsets},
                           std::pair{gathered.incidence, gathered.host_incidence}}) {
      error = cudaMemcpyAsync(util::ArenaPointer<std::uint32_t>(arena, pair.first),
          util::ArenaPointer<std::uint32_t>(gather_host.data(), pair.second),
          pair.first.bytes, cudaMemcpyHostToDevice, stream);
      if (error != cudaSuccess) return error;
    }
  }
""")
    elif path == "lib_src/solvers/NodalCinStorage.h":
        text = replace_once(text, """  util::HostArena gather_host;
  cin_advance::force_gather::View force_gather;
""")
    elif path == "lib_src/solvers/cin_advance/ForceTransfers.cu":
        text = replace_once(text, """  if (!blockIdx.x && !threadIdx.x && force_gather::Eligible(input.model, input.force_gather))
    *input.force_gather.summary = {};
""")
    elif path == "lib_src/solvers/cin_advance/Input.h":
        text = replace_once(text, '#include "ForceGatherTypes.h"\n')
        text = replace_once(text, "  force_gather::View force_gather;\n")
    elif path == "lib_utils/BUILD.bazel":
        text = replace_once(text, """
cc_library(
    name = "ordered_node_incidence",
    hdrs = ["OrderedNodeIncidence.h"],
)
""")
    else:
        raise AssertionError(path)
    return original(path, text)


def prove() -> None:
    for path in BASELINE:
        restore(path, (ROOT / path).read_text())
    references = {
        "ExplicitNodalCinStep.cu": (10568, "7c07918c54ac3fdeb4ea1734098617289193ce9beb7e5c4b8c76f3eebcb1e13f"),
        "CinForceTransfer.h": (5435, "1e9745039412de8a24212aa28256769685d0edbfe9bc7808c59af1730eaa9516"),
        "Incidence.h": (1912, "4bfc856ff5f5eff80bc5e4fddd0908aae5cd5a7912c34319ff8273a24fc531c9"),
    }
    for name, (size, digest) in references.items():
        data = (HERE / "reference" / name).read_bytes()
        assert len(data) == size and hashlib.sha256(data).hexdigest() == digest, name

    old_incidence = (HERE / "reference/Incidence.h").read_text()
    helper = (ROOT / "lib_utils/OrderedNodeIncidence.h").read_text()
    old_operations = body(old_incidence, "BuildIncidence")
    old_operations = old_operations[old_operations.index(
        "  for (std::size_t node = 0; node <= nodes; ++node"):]
    new_operations = body(helper, "BuildOrderedNodeIncidence")
    assert new_operations.index("if (node_at(parent, slot) >= nodes) return false;") < \
        new_operations.index("offsets[node] = 0")
    new_operations = new_operations[new_operations.index(
        "  for (std::size_t node = 0; node <= nodes; ++node"):]
    new_operations = new_operations.replace(
        "node_at(parent, slot)", "elements[parent].nodes[slot]")
    assert compact(new_operations).replace("{", "").replace("}", "") == \
        compact(old_operations).replace("{", "").replace("}", "")

    values = (ROOT / "lib_src/solvers/cin_advance/ForceGatherValues.h").read_text()
    gather = body(values, "GatherMaster")
    assert gather.index("!prepared[row].report") < gather.index(
        "const auto force = prepared[row].transferred_load.force[slot];")
    ordered = [
        "next.force[0] = next.force[0] + force.x;",
        "next.force[1] = next.force[1] + force.y;",
        "next.force[2] = next.force[2] + force.z;",
        "next.mass = next.mass + coefficient.mass;",
        "next.stiffness = next.stiffness + coefficient.translational_stiffness;",
        "next.inertia = next.inertia + coefficient.inertia;",
    ]
    positions = [gather.index(line) for line in ordered]
    assert positions == sorted(positions)
    assert gather.index("row != previous / 4 && !ValidMaster(next)") < positions[0]
    assert gather.rindex("if (!ValidMaster(next)) return false;") > positions[-1]
    numerical = body(values, "NumericalMass")
    expected = """  double next = *trial.numerical_mass;
  for (std::uint32_t row = 0; row < source.row_count; ++row) {
    if (!prepared[row].report) return false;
    next = next + 4 * prepared[row].transferred_coefficients.master[0].mass
        - prepared[row].secondary_mass;
    if (!constraints::tied_shell::detail::math::Finite(next)) return false;
  }
  output = next;
  return true;
"""
    assert compact(numerical) == compact(expected)
    apply = (ROOT / "lib_src/constraints/tied_shell/runtime/CinForceTransfer.h").read_text()
    apply_body = body(apply, "ApplyForceRow")
    secondary = body(values, "PublishSecondary")
    for expression in [
        "if (trial.mass[secondary] != 0) trial.saved_secondary_mass[row] = trial.mass[secondary];",
        "if (trial.inertia[secondary] != 0) trial.saved_secondary_inertia[row] = trial.inertia[secondary];",
        "trial.mass[secondary] = 0;",
        "trial.inertia[secondary] = 0;",
        "trial.translational_stiffness[secondary] = 1e-20;",
        "trial.rotational_stiffness[secondary] = 1e-20;",
        "trial.patches[row] = prepared[row].patch;",
    ]:
        assert compact(expression) in compact(secondary)
    assert compact("*trial.numerical_mass = *trial.numerical_mass + "
                   "4*transferred_coefficients.master[0].mass-secondary_mass;") in compact(apply_body)

    resolver = (ROOT / "lib_src/solvers/cin_advance/ForceGather.h").read_text()
    resolve = body(resolver, "Resolve")
    assert resolve.index("summary.mode == Mode::Staging") < resolve.index(
        "summary.mode = Mode::Publish")
    assert resolve.index("summary.report = force_transfers::Apply(input);") < resolve.index(
        "summary.mode = Mode::SerialCompleted")
    kernels = (ROOT / "lib_src/solvers/cin_advance/ForceGather.cu").read_text()
    assert "atomicAdd" not in kernels and "cudaMalloc" not in kernels and \
        "cudaMemcpy" not in kernels and "cudaStreamSynchronize" not in kernels
    assert "atomicExch(&view.summary->mode" in body(kernels, "Gather")
    publication = body(kernels, "PublishDestinations")
    first_write = min(publication.index("PublishMaster("),
                      publication.index("PublishSecondary("),
                      publication.index("*force.numerical_mass"))
    assert publication.index("summary->mode != Mode::Publish") < first_write
    launch = body(kernels, "Launch")
    assert launch.index("Gather<<<") < launch.index("Complete<<<") < \
        launch.index("PublishDestinations<<<")

    transfers = (ROOT / "lib_src/solvers/cin_advance/ForceTransfers.cu").read_text()
    prepare = body(transfers, "Prepare")
    assert prepare.index("*input.force_gather.summary = {};") < \
        prepare.index("force_inputs::ForceView(input)") < \
        prepare.index("PrepareForceRow(input.model")
    owner = (ROOT / "lib_src/solvers/ExplicitNodalCinStep.cu").read_text()
    owner_launch = body(owner, "cin_advance::Launch")
    assert owner_launch.index("force_transfers::Launch") < \
        owner_launch.index("force_gather::Launch") < \
        owner_launch.index("PrepareCin<<<")
    assert "transfers_gathered ? input.force_gather.summary->report" in body(owner, "PrepareCin")

    startup = (ROOT / "lib_src/solvers/NodalCinStartup.cpp").read_text()
    assert startup.index("if (groups || binding)") < startup.index(
        "if (layout.gather.device_bytes)")
    layout = (ROOT / "lib_src/solvers/NodalCinGatherLayout.h").read_text()
    assert layout.index("CinLayout& cin, StateLayout& owner") < layout.index(
        "cin.gather = gathered;")
    assert layout.index("CinOwnerHostFits(") < layout.index("cin.gather = gathered;")


if __name__ == "__main__":
    prove()
