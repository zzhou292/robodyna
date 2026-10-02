using System;

internal static class PointerChecks
{
    public static void Run()
    {
        // frompointer borrows primitive storage; it does not extend its lifetime.
        using (var owner = new double_ptr())
        {
            owner.assign(1.25);
            using (var alias = double_ptr.frompointer(owner.cast()))
            {
                Checks.Near(alias.value(), 1.25, 0, "Borrowed double value");
                alias.assign(-2.5);
                alias.Dispose(); alias.Dispose();
            }
            GC.Collect(); GC.WaitForPendingFinalizers();
            Checks.Near(owner.value(), -2.5, 0, "Borrowed double disposal preserves owner");
            GC.KeepAlive(owner);
        }
        using (var owner = new float_ptr())
        {
            owner.assign(1.5f);
            using (var alias = float_ptr.frompointer(owner.cast()))
            {
                Checks.Near(alias.value(), 1.5, 0, "Borrowed float value");
                alias.assign(-2.25f);
                alias.Dispose(); alias.Dispose();
            }
            GC.Collect(); GC.WaitForPendingFinalizers();
            Checks.Near(owner.value(), -2.25, 0, "Borrowed float disposal preserves owner");
            GC.KeepAlive(owner);
        }
        using (var owner = new int_ptr())
        {
            owner.assign(17);
            using (var alias = int_ptr.frompointer(owner.cast()))
            {
                Checks.Require(alias.value() == 17, "Borrowed integer value");
                alias.assign(-23);
                alias.Dispose(); alias.Dispose();
            }
            GC.Collect(); GC.WaitForPendingFinalizers();
            Checks.Require(owner.value() == -23, "Borrowed integer disposal preserves owner");
            GC.KeepAlive(owner);
        }
        using (var values = new vector_double())
        {
            values.Add(1.25); values.Add(2.5);
            using (var copy = new vector_double(values))
            {
                values[0] = -1;
                Checks.Near(copy[0], 1.25, 0, "Native double vector copy owns its values");
                copy[1] = -3;
                Checks.Near(values[1], 2.5, 0, "Native double vector mutation isolation");
            }
            bool rejected = false;
            try { double invalid = values[100]; }
            catch (ArgumentOutOfRangeException) { rejected = true; }
            Checks.Require(rejected, "Selected double vector owner lost its exception path");
        }
    }
}
