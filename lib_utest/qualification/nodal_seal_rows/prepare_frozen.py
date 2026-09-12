#!/usr/bin/env python3
"""Namespace-only adapters for complete, byte-authenticated 301a750 donors."""
import pathlib
import sys

here = pathlib.Path(__file__).resolve().parent
output = pathlib.Path(sys.argv[1])
output.mkdir(parents=True, exist_ok=True)
stability = (here / "frozen/ExplicitStepStability.h").read_text()
stability = stability.replace("tl::fea::stability", "tl::fea::seal_frozen_stability")
(output / "FrozenStability.h").write_text(stability)
caller = (here / "frozen/Validation.cuh").read_text()
caller = caller.replace('#include "../FENodalStateStorage.h"', '#include "FrozenTypes.h"')
caller = caller.replace("tl::fea::nodal_seal", "tl::fea::seal_frozen")
caller = caller.replace("using nodal_detail::Control;", "using Control = seal_test::FrozenControl;")
caller = caller.replace("stability::", "seal_frozen_stability::")
(output / "FrozenValidation.cuh").write_text(caller)
