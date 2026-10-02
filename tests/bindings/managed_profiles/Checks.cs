using System;
using System.IO;
using System.Collections.Generic;

internal static class Checks
{
    internal static void Require(bool value, string message)
    {
        if (!value) throw new Exception(message);
    }
    internal static void Near(double actual, double expected, double tolerance, string message)
    {
        Require(!Double.IsNaN(actual) && !Double.IsInfinity(actual) && Math.Abs(actual - expected) <= tolerance, message);
    }
    internal static void Owner(string basename)
    {
        var owners = new HashSet<string>();
        foreach (string line in File.ReadAllLines("/proc/self/maps"))
        {
            string[] fields = line.Split(new char[] {' '}, StringSplitOptions.RemoveEmptyEntries);
            if (fields.Length >= 6 && fields[fields.Length - 1].EndsWith("/" + basename))
                owners.Add(fields[3] + ":" + fields[4]);  // device/inode, not alias spelling
        }
        Require(owners.Count == 1, "Expected exactly one actual native owner: " + basename);
    }
}
