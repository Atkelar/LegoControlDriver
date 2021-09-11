using System;

namespace Atkelar.LegoControlII
{
    /// <summary>
    /// Event arguments for the servo finished event.
    /// </summary>
    public class ServoFinishedEventArgs
        : EventArgs
    {
        public ServoFinishedEventArgs(int motorChannel)
        {
            MotorChannel = motorChannel;
        }

        /// <summary>
        /// Gets the motor channel index (0,1,2) that just finished moving.
        /// </summary>
        public int MotorChannel { get; }
    }
}