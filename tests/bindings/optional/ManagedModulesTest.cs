using System;
using System.IO;
using System.Collections.Generic;

internal static class ManagedModulesTest
{
    private static void Require(bool value, string message)
    {
        if (!value) throw new Exception(message);
    }

    private static void Near(double actual, double expected, double tolerance, string message)
    {
        Require(!Double.IsNaN(actual) && !Double.IsInfinity(actual) &&
                Math.Abs(actual - expected) <= tolerance, message);
    }

    private static void CheckLoadedOwner(string basename)
    {
        var names = new HashSet<string>();
        foreach (string line in File.ReadAllLines("/proc/self/maps"))
        {
            string[] fields = line.Split(new char[] {' '}, StringSplitOptions.RemoveEmptyEntries);
            if (fields.Length > 0 && fields[fields.Length - 1].EndsWith("/" + basename))
                names.Add(fields[fields.Length - 1]);
        }
        Require(names.Count == 1, "Expected exactly one loaded owner: " + basename);
    }

    public static void Main()
    {
        using (var system = new ChSystemNSC())
        using (var body = new ChBody())
        using (var gravity = new ChVector3d(0, 0, 0))
        using (var velocity = new ChVector3d(1, 0, 0))
        {
            system.SetGravitationalAcceleration(gravity);
            body.SetMass(1);
            body.SetPosDt(velocity);
            system.AddBody(body);
            Require(system.DoStepDynamics(.01) != 0, "Real core step failed");
            using (var position = body.GetPos())
                Near(position.x, .01, 1e-12, "Body movement differs from free translation");
            Near(system.GetChTime(), .01, 1e-14, "Native clock mismatch");

            using (var terrain = new FlatTerrain(2.5, .7f))
            using (var point = new ChVector3d(1, 2, 8))
            {
                Near(terrain.GetHeight(point), 2.5, 1e-14, "Cross-module vector/terrain call failed");
                Near(terrain.GetCoefficientFriction(point), .7, 1e-6, "Terrain material mismatch");
            }
            using (var tire = new HMMWV_RigidTire("managed-tire"))
            using (var inertia = tire.GetTireInertia())
            {
                Near(tire.GetRadius(), .467, 1e-14, "Vehicle model radius mismatch");
                Near(inertia.y, 6.69, 1e-14, "Vehicle model vector return mismatch");
            }
            using (var exporter = new ChPovRay(system))
            {
                exporter.AddAll();
                exporter.RemoveAll();
            }
            // Construction and shared-pointer casting only: no window, display,
            // Vulkan device or second simulation owner is initialized.
            using (var visual = new ChVisualSystemVSG())
            using (var cast = chrono_vsg.CastToChVisualSystemVSG(visual))
                Require(cast != null, "Cross-module visual shared pointer cast failed");
        }
        foreach (string name in new string[] {"librobodyna_core.so", "librobodyna_native_images.so",
                 "librobodyna_native_vehicle.so", "librobodyna_native_vsg.so", "librobodyna_native_postprocess.so"})
            CheckLoadedOwner(name);
        Console.WriteLine("ROBODYNA MANAGED MODULES PASS");
    }
}
