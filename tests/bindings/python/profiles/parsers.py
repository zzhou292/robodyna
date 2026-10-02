"""Actual parser-created System and cross-module body ownership, no rendering."""

import gc
from pathlib import Path
import sys

# The declared isolated interpreter omits script directories from sys.path.
sys.path.insert(0, str(Path(__file__).parent))
from common import finish, open_package, require

product, manifest, document = open_package()
core, parsers = product.core, product.parsers
parser = parsers.ChParserMbsYAML(False)
# The retained documented no-file path returns a real default ChSystemNSC.
system = parser.CreateSystem()
system.SetNumThreads(1, 1, 1)
system.SetGravitationalAcceleration(core.ChVector3d(0, 0, 0))
body = core.ChBody()
body.SetMass(2.0)
body.SetPosDt(core.ChVector3d(1, 0, 0))
system.AddBody(body)
require(system.GetBodies()[0].GetMass() == 2.0, "Parser-created system rejects real core body")
require(system.DoStepDynamics(0.001), "Parser-created system failed a native physical step")
require(abs(system.GetChTime() - 0.001) < 1e-15, "Parser/core clock mismatch")
require(abs(body.GetPos().x - 0.001) < 1e-12, "Parser/core physical state exchange failed")
system.RemoveBody(body)
del parser, system
gc.collect()
require(body.GetMass() == 2.0, "Detached body lost shared ownership")
finish(manifest, document, ["real parser-created System", "one native step with core body", "detached body lifetime"], steps=1)
