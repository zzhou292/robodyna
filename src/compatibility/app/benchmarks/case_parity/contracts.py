"""Required resolved domains. An unknown declaration never authorizes matching."""
from dataclasses import dataclass
import json

from .artifacts import Artifacts, keys, require, text

DOMAINS = (
    "topology_geometry", "population", "formulations_materials_units",
    "mass_inertia", "constraints", "initial_loads_wall", "contact_history",
    "timestep_mass_control", "recorded_time_grid", "output_work_precision",
)
SCOPES = ("synthetic", "normal_response_packet", "selected_vehicle", "full_vehicle")


@dataclass(frozen=True)
class Contract:
    artifact: object
    case_id: str
    scope: str
    domains: dict
    numerical_protocol: object


def canonical(value):
    # Preserve signed zero and integer/float/bool distinctions; object order is not physics.
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def read_contract(pin, parent, artifacts=None):
    artifacts = artifacts if artifacts is not None else Artifacts()
    file = artifacts.verify(pin, parent)
    value = artifacts.object(file)
    keys(value, ("schema", "case_id", "scope", "domains", "numerical_protocol"), "case contract")
    require(value["schema"] == "robo_dyna.resolved_case_contract.v1", "unknown contract schema")
    case_id = text(value["case_id"], "case ID")
    require(value["scope"] in SCOPES, "unsupported comparison scope")
    keys(value["domains"], DOMAINS, "resolved domains")
    for name in DOMAINS:
        domain = value["domains"][name]
        keys(domain, ("status", "definition", "evidence"), name)
        require(domain["status"] in ("resolved", "absent", "unknown"), f"{name}: invalid resolution status")
        require(isinstance(domain["definition"], dict) and domain["definition"],
                f"{name}: explicit definition or unresolved reason required")
        require(isinstance(domain["evidence"], list) and len(domain["evidence"]) <= 16,
                f"{name}: evidence list exceeds cap")
        require(domain["status"] == "unknown" or domain["evidence"],
                f"{name}: resolved/absent declaration needs pinned evidence")
        for evidence in domain["evidence"]:
            artifacts.verify(evidence, file.path.parent)
    time = value["domains"]["recorded_time_grid"]
    if time["status"] != "unknown":
        require(time["status"] == "resolved", "time grid cannot be absent")
        definition = time["definition"]
        keys(definition, ("kind", "planned_steps", "initial_time_s", "requested_end_time_s"), "time definition")
        require(definition["kind"] in ("physical_steps", "prescribed_updates"), "unknown time-grid kind")
        require(type(definition["planned_steps"]) is int and 0 < definition["planned_steps"] <= 10000000,
                "bounded positive declared step count required")
        for name in ("initial_time_s", "requested_end_time_s"):
            require(type(definition[name]) in (int, float), "numerical declared times required")
        require(definition["initial_time_s"] >= 0 and definition["requested_end_time_s"] >= definition["initial_time_s"],
                "invalid declared time interval")
        if value["scope"] in ("selected_vehicle", "full_vehicle"):
            require(definition["kind"] == "physical_steps", "vehicle benchmark requires physical steps")
        if definition["kind"] == "physical_steps":
            require(definition["requested_end_time_s"] > definition["initial_time_s"],
                    "initial-only physical case cannot qualify")
    output = value["domains"]["output_work_precision"]
    if output["status"] != "unknown":
        require(output["status"] == "resolved", "output work cannot be absent")
        definition = output["definition"]
        keys(definition, ("precision", "format", "fields", "sample_epochs"), "output definition")
        require(definition["precision"] == "binary64" and definition["format"] == "case_fields_f64_v1",
                "unsupported shared output format/precision")
        fields = definition["fields"]
        require(isinstance(fields, list) and 0 < len(fields) <= 32, "bounded output fields required")
        names = set()
        for field in fields:
            keys(field, ("name", "components"), "output field")
            text(field["name"], "output field name")
            require(field["name"] not in names and type(field["components"]) is int and field["components"] > 0,
                    "duplicate field or invalid component count")
            names.add(field["name"])
        epochs = definition["sample_epochs"]
        require(isinstance(epochs, list) and 2 <= len(epochs) <= 1000 and
                all(type(n) is int and n >= 0 for n in epochs) and epochs[0] == 0 and
                all(b > a for a,b in zip(epochs,epochs[1:])), "invalid output sample grid")
        if time["status"] == "resolved":
            require(epochs[-1] == time["definition"]["planned_steps"], "output does not reach requested horizon")
    protocol = value["numerical_protocol"]
    if protocol is not None:
        keys(protocol, ("id", "test_name", "tolerances"), "numerical protocol")
        text(protocol["id"], "numerical protocol ID")
        name = text(protocol["test_name"], "numerical test name")
        require(name.count(".") == 1 and not any(c in name for c in "*: \n\t"),
                "numerical evidence needs one exact named test")
        artifacts.verify(protocol["tolerances"], file.path.parent)
    return Contract(file, case_id, value["scope"], value["domains"], protocol)


def compare_contracts(reference, candidate):
    differences = []
    if reference.scope != candidate.scope:
        differences.append({"domain": "scope", "reason": "different_declared_scope",
                            "reference": reference.scope, "candidate": candidate.scope})
    for name in DOMAINS:
        a, b = reference.domains[name], candidate.domains[name]
        if a["status"] == "unknown" or b["status"] == "unknown":
            differences.append({"domain": name, "reason": "unresolved_evidence"})
        if a["status"] != b["status"] or canonical(a["definition"]) != canonical(b["definition"]):
            differences.append({"domain": name, "reason": "resolved_definition_differs",
                                "reference": a["definition"], "candidate": b["definition"]})
    # Protocol files can live in distinct directories; compare semantics and actual content pins.
    a, b = reference.numerical_protocol, candidate.numerical_protocol
    if a is not None and b is not None:
        if (a["id"], a["test_name"], a["tolerances"]["sha256"]) != (
                b["id"], b["test_name"], b["tolerances"]["sha256"]):
            differences.append({"domain": "numerical_protocol", "reason": "different_comparison_protocol"})
    return differences
