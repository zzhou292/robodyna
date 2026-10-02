using System;

internal static class RobotShapeChecks
{
    public static void Run()
    {
        Checks.Require(typeof(BoxShape) != typeof(RoboSimianBoxShape), "Distinct native box types were collapsed");
        Checks.Require(typeof(CylinderShape) != typeof(RoboSimianCylinderShape), "Distinct native cylinder types were collapsed");
        Checks.Require(typeof(SphereShape) != typeof(RoboSimianSphereShape), "Distinct native sphere types were collapsed");
        using (var position = new ChVector3d(1, 2, 3))
        using (var rotation = new ChQuaterniond(1, 0, 0, 0))
        using (var dimensions = new ChVector3d(4, 5, 6))
        {
            using (var core = new BoxShape(position, rotation, dimensions, 7))
            using (var robot = new RoboSimianBoxShape(position, rotation, dimensions))
            {
                Checks.Require(core.matID == 7, "Core box constructor lost material identity");
                Checks.Near(robot.m_dims.z, 6, 0, "RoboSimian box dimensions");
                using (var offset = robot.m_pos) offset.x = 10;
                Checks.Near(robot.m_pos.x, 10, 0, "RoboSimian box field mutation");
                Checks.Near(core.pos.x, 1, 0, "Distinct Core box storage");
                robot.Dispose(); robot.Dispose();
                Checks.Near(core.dims.y, 5, 0, "Core box remains valid after Robot disposal");
            }
            using (var core = new CylinderShape(position, rotation, 1.5, 2.5, 8))
            using (var robot = new RoboSimianCylinderShape(position, rotation, 4.5, 5.5))
            {
                Checks.Require(core.matID == 8, "Core cylinder constructor lost material identity");
                Checks.Near(robot.m_radius, 4.5, 0, "RoboSimian cylinder radius");
                robot.m_length = 6.5;
                Checks.Near(robot.m_length, 6.5, 0, "RoboSimian cylinder field mutation");
                core.Dispose(); core.Dispose();
                Checks.Near(robot.m_pos.y, 2, 0, "Robot cylinder remains valid after Core disposal");
            }
            using (var core = new SphereShape(position, 2.5, 9))
            using (var robot = new RoboSimianSphereShape(position, 3.5))
            {
                Checks.Require(core.matID == 9, "Core sphere constructor lost material identity");
                robot.m_radius = 4.5;
                Checks.Near(robot.m_radius, 4.5, 0, "RoboSimian sphere field mutation");
                Checks.Near(core.radius, 2.5, 0, "Distinct Core sphere storage");
                robot.Dispose(); robot.Dispose();
                Checks.Near(core.pos.z, 3, 0, "Core sphere remains valid after Robot disposal");
            }
        }
        using (var values = new vector_int())
        {
            values.Add(11); values.Add(22);
            using (var copy = new vector_int(values))
            {
                values[0] = -11;
                Checks.Require(copy[0] == 11, "Native integer vector copy owns its values");
                copy[1] = -22;
                Checks.Require(values[1] == 22, "Native integer vector mutation isolation");
            }
            bool rejected = false;
            try { int invalid = values[100]; }
            catch (ArgumentOutOfRangeException) { rejected = true; }
            Checks.Require(rejected, "Selected integer vector owner lost its exception path");
        }
    }
}
