"""Host Sensor/body ownership through actual OptiX-profile native wrappers."""

import gc
from pathlib import Path
import sys

# The declared isolated interpreter omits script directories from sys.path.
sys.path.insert(0, str(Path(__file__).parent))
from common import finish, open_package, require

product, manifest, document = open_package()
core, sensor = product.core, product.sensor
body = core.ChBody()
body.SetMass(3.5)
system = core.ChSystemSMC()
system.SetNumThreads(1, 1, 1)
system.AddBody(body)
reference = core.ChVector3d(-89.4, 43.1, 260)
gps = sensor.ChGPSSensor(body, 20.0, core.ChFramed(), reference, sensor.ChNoiseNone())
require(gps.GetUpdateRate() == 20.0, "GPS rate changed through wrapper")
require(gps.GetGPSReference().z == 260.0, "Core vector did not cross Sensor boundary")
gps.GetParent().SetMass(8.25)
require(body.GetMass() == 8.25, "Sensor and Core do not share their actual body")
system.RemoveBody(body)
del body, reference
gc.collect()
require(gps.GetParent().GetMass() == 8.25, "Sensor lost shared parent lifetime")
buffer = sensor.UserGPSBuffer()
require(not buffer.HasData(), "Fresh GPS buffer unexpectedly contains samples")
finish(manifest, document, ["actual Sensor/core body exchange", "GPS parent shared lifetime", "real empty Sensor buffer"])
