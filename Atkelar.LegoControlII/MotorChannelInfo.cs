using System;

namespace Atkelar.LegoControlII
{
    /// <summary>
    /// Defines a motor channel setup.
    /// </summary>
    public class MotorChannelInfo
    {
        /// <summary>
        /// Create a new instance of a motor channel setup.
        /// </summary>
        /// <param name="channel">The motor channel as seen on the Lego controller.</param>
        /// <param name="calibrationString">The calibration string for the speed data.</param>
        /// <param name="inputChannel">The mapped input channel, either 6 or 7, for this motor channel. Null if not mapped.</param>
        public MotorChannelInfo(MotorChannel channel, string calibrationString, byte? inputChannel)
        {
            if (channel < MotorChannel.A || channel > MotorChannel.C)
                throw new ArgumentOutOfRangeException(nameof(channel), channel, "Only A-C are supported for motor channels!");
            if (inputChannel.HasValue && (inputChannel.Value < 6 || inputChannel.Value > 7))
                throw new ArgumentOutOfRangeException(nameof(inputChannel), inputChannel, "Only 6 and 7 are supported for input channels!");

            Channel = channel;
            MappedInput = inputChannel;
            _CalibrationString = calibrationString;
        }

        /// <summary>
        /// Gets the assigned motor channel name.
        /// </summary>
        public MotorChannel Channel { get; }
        /// <summary>
        /// Gets the assigned input channel number.
        /// </summary>
        public byte? MappedInput { get; }

        private string _CalibrationString;

        internal string GetCalibrationString()
        {
            return _CalibrationString;
        }
    }
}