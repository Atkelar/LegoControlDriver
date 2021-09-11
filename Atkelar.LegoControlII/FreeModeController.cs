using System;
using System.Text;

namespace Atkelar.LegoControlII
{
    /// <summary>
    /// Provides a simple interface for "free form" movement commands.
    /// </summary>
    public class FreeModeController
        : IDisposable
    {
        private LegoController legoController;
        private Direction[] Directions;
        private byte[] Speeds;

        internal FreeModeController(LegoController legoController)
        {
            this.legoController = legoController;
            Directions = new Direction[legoController.ConfiguredMotorChannels];
            Speeds = new byte[legoController.ConfiguredMotorChannels];
            legoController.ServoFinished += ServoFinishedHandler;
        }

        public void Dispose()
        {
            if (legoController != null)
            {
                if (legoController.IsOpen)
                {
                    FullStop();
                    legoController.EndMode(OperationalMode.Free);
                }
                legoController.ServoFinished -= ServoFinishedHandler;
            }
            legoController = null;
        }

        private void ServoFinishedHandler(object sender, ServoFinishedEventArgs e)
        {
            lock (this)
            {
                ServoRuning[e.MotorChannel] = false;
                _ServoChangedTrigger.Set();
            }
        }

        private bool[] ServoRuning = new bool[3];
        private System.Threading.AutoResetEvent _ServoChangedTrigger = new System.Threading.AutoResetEvent(false);

        /// <summary>
        /// Stops all movement in the controller, i.e. all motor channels are set to stop and digital outputs set to zero.
        /// </summary>
        public void FullStop()
        {
            for (byte i = 0; i < legoController.ConfiguredMotorChannels; i++)
                Motor(i);
            legoController.SendCommandAndHandleGenericResponse("set:------");  // clear all outputs if possible.
        }

        /// <summary>
        /// Gets the current direction of a motor channel.
        /// </summary>
        /// <param name="motorchannel">The channel index, 0,1 or 2.</param>
        /// <returns>The direction that the motor is currently moving in.</returns>
        public Direction CurrentMotorDirection(byte motorchannel)
        {
            return Directions[motorchannel];
        }

        /// <summary>
        /// If the direction is the same as the current one, the result is a "stop".
        /// </summary>
        /// <param name="motorchannel">The motor channel, 0,1 or 2.</param>
        /// <param name="direction">The new requested direction.</param>
        /// <param name="speed">The speed (1-100) to use.</param>
        public void ReverseOrStop(byte motorchannel, Direction direction = Direction.Stop, byte speed = 100)
        {
            if (direction == Directions[motorchannel])
                direction = Direction.Stop;
            Motor(motorchannel, direction, speed);
        }

        /// <summary>
        /// Run the motor in a specific direction.
        /// </summary>
        /// <param name="motorchannel">The motor channel, 0,1 or 2.</param>
        /// <param name="direction">The new requested direction.</param>
        /// <param name="speed">The speed (1-100) to use.</param>
        public void Motor(byte motorchannel, Direction direction = Direction.Stop, byte speed = 100)
        {
            if (speed > 100)
                speed = 100;
            if (speed < 1)
                direction = Direction.Stop;
            if (motorchannel >= Directions.Length)
                throw new ArgumentOutOfRangeException(nameof(motorchannel), motorchannel, string.Format("The controller only has {0} motor channels configured!", Directions.Length));
            string cmd;
            if (direction == Direction.Stop)
                cmd = string.Format("drv:{0}:s", motorchannel);
            else
                cmd = string.Format("drv:{0}:{1}:{2:x2}", motorchannel, direction == Direction.Forward ? "f" : "r", speed);
            Directions[motorchannel] = direction;
            Speeds[motorchannel] = direction == Direction.Stop ? (byte)0 : speed;
            legoController.SendCommandAndHandleGenericResponse(cmd);
        }

        /// <summary>
        /// Runs the specified motor channel in the direction of choice, until the number of ticks are reached.
        /// </summary>
        /// <param name="motorChannel">The motor channel to move, 0,1 or 2. The channel has to be configured with a counter.</param>
        /// <param name="direction">The direction to move. If this is <see cref="Direction.Stop"/> the <paramref name="ticks"/> needs to be zero!</param>
        /// <param name="ticks">The number of ticks to move. If this is non-zero, the direction has to be either <see cref="Direction.Forward"/> or <see cref="Direction.Reverse"/></param>
        /// <param name="speed">The speed in percent to use. 1-100.</param>
        public void ServoMotor(int motorChannel, Direction direction, int ticks, byte speed = 100)
        {
            if (motorChannel >= this.Directions.Length)
                throw new ArgumentOutOfRangeException(nameof(motorChannel), motorChannel, string.Format("Only {0} configured motor channels!", this.Directions.Length));
            if (!legoController.MotorHasCounter(motorChannel))
                throw new ArgumentOutOfRangeException(nameof(motorChannel), motorChannel, "The channel has no counter attached!");
            if (speed == 0)
                direction = Direction.Stop;
            if (speed > 100)
                speed = 100;
            if (ticks < 0)
                throw new ArgumentOutOfRangeException(nameof(ticks), ticks, "Can only count positive ticks!");
            if (direction == Direction.Stop)
            {
                if (ticks > 0)
                    throw new InvalidOperationException("Cannot 'stop' to non-zero ticks!");
                legoController.SendCommandAndHandleGenericResponse(string.Format("drv:{0}:S", motorChannel));
            }
            else
            {
                if (ticks == 0)
                    throw new InvalidOperationException("Cannot 'move' for zero ticks!");
                lock (this)
                {
                    ServoRuning[motorChannel] = true;
                }
                legoController.SendCommandAndHandleGenericResponse(string.Format("srv:{0}:{1}:{2:x4}:{3:x2}", motorChannel, direction == Direction.Forward ? 'F' : 'R', ticks, speed));
            }
        }

        /// <summary>
        /// Waits (blocks) until the specified channels are done moving.
        /// </summary>
        /// <param name="channels">The channel number (0,1 or 2) to wait for.</param>
        public void WaitForServoFinished(params byte[] channels)
        {
            bool isRunning = true;
            while (isRunning)
            {
                isRunning = false;

                lock (this)
                {
                    for (int i = 0; i < channels.Length; i++)
                    {
                        if (legoController.MotorHasCounter(channels[i]))
                        {
                            if (ServoRuning[channels[i]])
                                isRunning = true;
                        }
                    }
                }
                if (isRunning)
                {
                    if (!_ServoChangedTrigger.WaitOne(5000)) // make sure we have an exit strategy
                    {
                        // Console.WriteLine("Uh-oh...");

                        isRunning = false;
                        for (int i = 0; i < channels.Length; i++)
                        {
                            if (legoController.MotorHasCounter(channels[i]))
                            {
                                //Console.WriteLine("{0}: {1}", channels[i], ServoRuning[channels[i]]);
                                //Console.WriteLine("Polling on channel {0}", channels[i]);
                                //Console.WriteLine(" .. sent {0}", channels[i]);
                                var response = legoController.SendCommandAndReadExectedResponse(string.Format("mstat:{0}", channels[i]), "mot:");
                                legoController.WaitForCommandComplete();
                                //Console.WriteLine(" -->{0}", response);
                                if (response == null)
                                {
                                    FullStop();
                                    throw new InvalidOperationException("Servo didn't report!");
                                }
                                var x = response.Split(':');
                                if (x.Length > 4)
                                {
                                    isRunning |= (ServoRuning[channels[i]] = x[4] == "*");
                                    // Console.WriteLine("{0}: {1} ({2})", channels[i], ServoRuning[channels[i]], response);
                                }
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Reads back the absolute values of the counters.
        /// </summary>
        /// <returns>The current counter values.</returns>
        /// <remarks>
        /// <para>This will not work if a servo operation is running on the channels. It will also reset the counters internally!</para>
        /// <para>Note that the sensors don't detect "directions", so any movement will increment the counters!</para>
        /// </remarks>
        public CounterValues? ReadAbsoluteCounters()
        {
            // expected response: cnt:x:y
            string response = legoController.SendCommandAndReadExectedResponse("qcnt", "cnt:");
            if (response != null)
            {
                //Console.WriteLine(response);
                var s = response.Split(':');
                if (s.Length != 3)
                    throw new InvalidOperationException("Unexpected counter respons!");
                legoController.WaitForCommandComplete();
                return new CounterValues() { Channel6 = uint.Parse(s[1]), Channel7 = uint.Parse(s[2]) };
            }

            return null;
        }
    }
}