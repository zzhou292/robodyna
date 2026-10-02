using System;

internal static class RosSensorTest
{
    public static void Main()
    {
        PointerChecks.Run();
        RobotShapeChecks.Run();
        using (var system = new ChSystemSMC())
        using (var body = new ChBody())
        using (var offset = new ChFramed())
        using (var reference = new ChVector3d(-89.4, 43.1, 260))
        using (var noise = new ChNoiseNone())
        {
            system.SetNumThreads(1, 1, 1);
            body.SetName("managed-parent");
            system.AddBody(body);
            using (var gps = new ChGPSSensor(body, 20.0f, offset, reference, noise))
            using (var handler = new ChROSGPSHandler(gps, "/robodyna/gps"))
            using (var tf = new ChROSTFHandler(100))
            {
                tf.AddSensor(gps, body.GetName(), "gps");
                Checks.Near(handler.GetUpdateRate(), 20, 1e-12, "ROS/Sensor shared-pointer argument");
                Checks.Require(handler.GetTickCount() == 0, "Handler unexpectedly executed transport");
                using (var parent = gps.GetParent()) parent.SetMass(4.75);
                Checks.Near(body.GetMass(), 4.75, 1e-12, "ROS/Sensor/Core body identity");
                // No bridge, sidecar or sensor manager is initialized. This is
                // actual managed/native ownership admission, not a ROS delivery.
            }
        }
        Checks.Require(typeof(ChROSHandler).IsAssignableFrom(typeof(ChROSCameraHandler)), "Camera handler declaration missing");
        Checks.Owner("librobodyna_core.so");
        Checks.Owner("librobodyna_native_sensor.so");
        Checks.Owner("librobodyna_native_ros.so");
        Checks.Owner("librobodyna_native_robot.so");
        Console.WriteLine("ROBODYNA MANAGED ROS SENSOR PASS");
    }
}
