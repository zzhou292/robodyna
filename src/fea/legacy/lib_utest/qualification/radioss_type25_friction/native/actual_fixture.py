"""Emit exact binary64 input literals; no output or mechanics is generated."""
import json
from pathlib import Path

def header(root):
    data = json.loads((root / "original/actual-force-input.json").read_text())["record"]
    row, profile = data["row"], data["profile"]
    assert data["stage"] == "FOR3_positive_entry" and data["observation_only"]
    assert data["native_units"] == "mm, tonne, s"
    for key, value in {"MFROT": 2, "IFQ": 10, "IORTHFRIC": 0, "INTTH": 0,
                       "IVIS2": 1, "INACTI": 5, "IGSTI": 4, "ALPHA0": 1, "STIGLO": -1}.items():
        assert profile[key] == value
    h = lambda v: float(v).hex()
    vec = lambda values: "{" + ", ".join(h(x) for x in values) + "}"
    common = profile["common"]
    old_p, old_k, old_f = row["PENE_OLD"], row["STIF_OLD"], row["SECND_FR"]
    code = ["// Generated exact input literals from pinned actual native observation.", "#pragma once",
            "namespace type25_friction_test {", "inline Case ActualForceInput() {", "  auto c = Basic();",
            "  c.input.normal.penetration = " + h(row["penetration"]) + ";",
            "  c.input.normal.stiffness = " + h(row["incoming_stiffness"]) + ";",
            "  c.input.normal.dt = " + h(common["DT1"]) + ";",
            "  c.input.normal.time = " + h(common["TT"]) + ";",
            "  c.input.dt12 = " + h(common["DT12"]) + ";",
            "  c.input.normal.secondary_mass = " + h(row["secondary_mass"]) + ";",
            "  c.input.normal_axis = " + vec(row["normal_axis"]) + ";",
            "  c.input.relative_velocity = " + vec(row["secondary_velocity"]) + ";"]
    for i in range(4):
        code += ["  c.input.normal.main_mass[%d] = %s;" % (i, h(row["main_masses"][i])),
                 "  c.input.normal.weights[%d] = %s;" % (i, h(row["weights"][i])),
                 "  c.input.main_vertices[%d] = %s;" % (i, vec(row["main_positions"][i]))]
        # Original left-to-right Vsecondary-H1*V1-H2*V2-H3*V3-H4*V4 order.
        for axis, name in enumerate("xyz"):
            code += ["  c.input.relative_velocity.%s = c.input.relative_velocity.%s - c.input.normal.weights[%d] * %s;" %
                     (name, name, i, h(row["main_velocities"][i][axis]))]
    code += ["  c.history.normal = " + vec([old_p[1], old_k[1], old_p[0], old_k[0], old_p[2]]) + ";",
             "  c.history.previous_force = " + vec(old_f[3:6]) + ";",
             "  c.history.staged_force = " + vec(old_f[0:3]) + ";",
             "  RefreshNormalVelocity(c);", "  return c;", "}", "} // namespace type25_friction_test"]
    return "\n".join(code) + "\n"
