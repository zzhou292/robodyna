"""Small real SolidWorks-style export used by the retained C++ import path."""
import pychrono as chrono

body = chrono.ChBodyAuxRef()
body.SetName("embedded_body")
body.SetMass(2)
body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body.SetPos(chrono.ChVector3d(0, 0, 2))
exported_items = [body]
