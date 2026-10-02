"""Actual ROS wrapper/core/director ownership, with no middleware subprocess."""

import gc
from pathlib import Path
import sys
import tempfile
import weakref

sys.path.insert(0, str(Path(__file__).parent.parent / "profiles"))
from common import finish, open_package, require

product, manifest, document = open_package()
core, ros = product.core, product.ros
system = core.ChSystemSMC()
system.SetNumThreads(1, 1, 1)
body = core.ChBody()
body.SetMass(5.0)
system.AddBody(body)
body_handler = ros.ChROSBodyHandler(25.0, body, "~/ownership")
require(body_handler.GetUpdateRate() == 25.0, "Real body handler constructor/rate changed")
tf = ros.ChROSTFHandler(20.0)
tf.AddTransform(body, "body", core.ChFramed(core.ChVector3d(1, 2, 3)), "local")
manager = ros.ChROSManager("robodyna_python_ownership")
require(not manager.IsInitialized(), "Host ownership gate unexpectedly initialized middleware")
manager.RegisterHandler(body_handler)
manager.RegisterHandler(tf)

class Handler(ros.ChROSHandler):
    def __init__(self):
        super().__init__(10.0)
    def Initialize(self, bridge):
        return True
    def Tick(self, time):
        pass

handler = Handler()
reference = weakref.ref(handler)
manager.RegisterHandler(handler)
del handler
gc.collect()
require(reference() is not None, "Native manager lost its registered Python director")
require(reference().GetUpdateRate() == 10.0, "Retained Python director/native object differs")
checks = ["real ROS/core body and frame exchange", "registered Python director lifetime"]

if document["profile"] == "ros_sensor":
    sensor = product.sensor
    gps = sensor.ChGPSSensor(body, 10, core.ChFramed(), core.ChVector3d(-89, 43, 100), sensor.ChNoiseNone())
    # Original Python API intentionally uses AddTransform, not the C# AddSensor.
    tf.AddTransform(gps.GetParent(), "body", gps.GetOffsetPose(), "gps")
    manager.RegisterHandler(ros.ChROSGPSHandler(gps, "~/gps"))
    checks.append("real Sensor/core/ROS parent and frame exchange")

if document["profile"] == "ros_urdf":
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "robot.urdf"
        path.write_text('''<robot name="python_ros_ownership">
<link name="world"/><link name="payload"><inertial><mass value="2"/>
<origin xyz="0 0 0"/><inertia ixx="1" iyy="2" izz="3" ixy="0" ixz="0" iyz="0"/>
</inertial></link><joint name="fixed" type="fixed"><parent link="world"/>
<child link="payload"/><origin xyz="1 2 3" rpy="0 0 0"/></joint></robot>''')
        parser = product.parsers.ChParserURDF(str(path))
        parser.PopulateSystem(system)
        tf.AddURDF(parser)
        manager.RegisterHandler(ros.ChROSRobotModelHandler(parser))
        require(parser.GetChBody("payload").GetMass() == 2.0, "URDF/core object exchange failed")
        checks.append("actual URDF parser and single native TF AddURDF implementation")

require(system.GetChTime() == 0.0, "Import/owner gate unexpectedly stepped the system")
require(not manager.IsInitialized(), "No transport qualification is claimed by this gate")
del manager
gc.collect()
require(reference() is None, "Manager destruction retained the Python director unexpectedly")
finish(manifest, document, checks + ["middleware transport remains a separate real-node gate"])
