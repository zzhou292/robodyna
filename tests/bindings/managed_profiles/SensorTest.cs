using System;

internal static class SensorTest
{
    public static void Main()
    {
        PointerChecks.Run();
        using (var system = new ChSystemSMC())
        using (var body = new ChBody())
        using (var offset = new ChFramed())
        using (var reference = new ChVector3d(-89.4, 43.1, 260))
        using (var noise = new ChNoiseNone())
        {
            system.SetNumThreads(1, 1, 1);
            body.SetMass(3.5);
            system.AddBody(body);
            using (var gps = new ChGPSSensor(body, 20.0f, offset, reference, noise))
            {
                Checks.Near(gps.GetUpdateRate(), 20, 1e-12, "Sensor update rate");
                using (var position = gps.GetGPSReference())
                    Checks.Near(position.z, 260, 1e-12, "Sensor/core vector return");
                using (var parent = gps.GetParent()) parent.SetMass(8.25);
                Checks.Near(body.GetMass(), 8.25, 1e-12, "Cross-module shared body identity");
                system.RemoveBody(body);
                body.Dispose();
                GC.Collect(); GC.WaitForPendingFinalizers();
                using (var parent = gps.GetParent())
                    Checks.Near(parent.GetMass(), 8.25, 1e-12, "Sensor parent shared lifetime");
            }
        }
        // Camera/OptiX constructors create CUDA streams. Check exposure without
        // constructing them in this CPU-only gate; GPU execution is separate.
        Checks.Require(typeof(ChSensor).IsAssignableFrom(typeof(ChCameraSensor)), "Camera native hierarchy not exposed");
        Checks.Owner("librobodyna_core.so");
        Checks.Owner("librobodyna_native_sensor.so");
        Console.WriteLine("ROBODYNA MANAGED SENSOR PASS");
    }
}
