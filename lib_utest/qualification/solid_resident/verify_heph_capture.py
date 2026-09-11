#!/usr/bin/env python3
"""Independent alpha=2 invariant check; no eigen/native/production code imports."""
from decimal import Decimal, localcontext
from pathlib import Path
import math
import re


def array(source, name, count):
    match = re.search(rf"\b{name}\{{(.*?)\}};", source, re.S)
    assert match, name
    tokens = match[1].replace("\n", " ").split(",")
    values = [float.fromhex(x.strip()) if "0x" in x else float(x) for x in tokens]
    assert len(values) == count and all(math.isfinite(x) for x in values), name
    return values


def invariant(strain, precision):
    with localcontext() as context:
        context.prec = precision
        d = Decimal.from_float
        # The existing prepared native bulk expression is rounded once to
        # binary64. The independent tensor calculation then uses exact inputs.
        mu = d(24e6)
        nu = .463
        bulk = d((24e6*2)*(1+nu)/(3*(1-2*nu)))
        a, b, c = [1+d(x) for x in strain[:3]]
        xy, yz, zx = [d(x)/2 for x in strain[3:]]
        determinant = a*(b*c-yz*yz)-xy*(xy*c-yz*zx)+zx*(xy*yz-b*zx)
        j = determinant.sqrt()
        shear = mu*((-Decimal(5)/3)*j.ln()).exp()
        pressure = bulk*(j-1)
        mean = (a+b+c)/3
        # For one alpha=2 term: sigma = mu J^(-5/3) dev(C)+K(J-1) I.
        return [shear*(a-mean)+pressure, shear*(b-mean)+pressure,
                shear*(c-mean)+pressure, shear*xy, shear*yz, shear*zx]


def main():
    source = Path(__file__).with_name("HephCapturedPacket.h").read_text()
    native = array(source, "Native", 187)
    expected = array(source, "InvariantStress", 6)
    position = array(source, "Position", 24)
    velocity = array(source, "Velocity", 24)
    assert position[15] == .04+velocity[15]*2**-20
    assert all(v == 0 for i, v in enumerate(velocity) if i != 15)
    a = invariant(native[173:179], 80)
    b = invariant(native[173:179], 120)
    assert [float(x) for x in a] == [float(x) for x in b] == expected
    for k, value in enumerate(b):
        print(f"stress[{k}] independent120={value} native_error_Pa="
              f"{Decimal.from_float(native[k])-value}")
    print("PASS: exact captured drift; 80/120-digit invariant stress rounds identically")


if __name__ == "__main__":
    main()
