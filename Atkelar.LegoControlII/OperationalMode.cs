using System;

namespace Atkelar.LegoControlII
{
    /// <summary>
    /// Describes the current mode of the controller object.
    /// </summary>
    [Flags]
    public enum OperationalMode
    {
        Unknown = 0,
        Disconnected = 1,
        Init = 2,
        Free = 4,
        Auto = 8,
        All = 0xf,
        Connected = Init | Free | Auto
    }
}