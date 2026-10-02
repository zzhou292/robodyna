using System;

internal static class OpenCrgTest
{
    public static void Main()
    {
        using (var system = new ChSystemSMC())
        using (var terrain = new CRGTerrain(system))
        using (var point = new ChVector3d(1, 0, 0))
        {
            system.SetNumThreads(1, 1, 1);
            terrain.UseMeshVisualization(false);
            terrain.Initialize("data/vehicle/terrain/crg_roads/handmade_straight.crg");
            Checks.Near(terrain.GetLength(), 22, 1e-12, "OpenCRG road length");
            Checks.Near(terrain.GetWidth(), 3, 1e-12, "OpenCRG road width");
            Checks.Near(terrain.GetHeight(point), .0111111, 1e-8, "Native OpenCRG height through managed core vector");
            using (var ground = terrain.GetGround())
                Checks.Require(Convert.ToBoolean(ground.IsFixed()), "Terrain ground ownership changed");
            Checks.Require(Convert.ToBoolean(system.DoStepDynamics(.001)), "Shared system step failed");
            Checks.Near(system.GetChTime(), .001, 1e-15, "Terrain/core clock mismatch");
        }
        Checks.Owner("librobodyna_core.so");
        Checks.Owner("librobodyna_native_vehicle.so");
        Console.WriteLine("ROBODYNA MANAGED OPENCRG PASS");
    }
}
