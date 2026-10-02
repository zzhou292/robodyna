using System;

/// <summary>Exercises real compiler code generation, documentation parsing and captured locals.</summary>
public static class LocalFunctions
{
    public static int Main()
    {
        int captured = 2;
        int Sum(int value) { return value + captured; }
        if (Sum(4) != 6) throw new Exception("Local-function capture failed");
        captured = 3;
        if (Sum(4) != 7) throw new Exception("Updated local-function capture failed");
        Console.WriteLine("ROBODYNA COMPILER ADMISSION PASS");
        return 0;
    }
}
