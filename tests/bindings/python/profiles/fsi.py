"""Host-side SPH callback/core values using the actual CUDA-capable module."""

import gc
from pathlib import Path
import sys

# The declared isolated interpreter omits script directories from sys.path.
sys.path.insert(0, str(Path(__file__).parent))
from common import finish, open_package, require

product, manifest, document = open_package()
core, fsi = product.core, product.fsi
callback = fsi.ParticlePropertiesCallback()
velocity = core.ChVector3d(1, 2, 3)
callback.v0 = velocity
velocity.x = 99
require(callback.v0.x == 1, "SPH callback no longer owns its copied core vector")
del velocity
gc.collect()
require(callback.v0.z == 3, "Callback/core value lifetime failed")
callback.rho0 = 1000.0
require(callback.rho0 == 1000.0, "Actual SPH callback state not exposed")
finish(manifest, document, ["actual SPH callback/core vector exchange and copy lifetime",
                            "GPU solver initialization/stepping is a separate guarded gate"])
