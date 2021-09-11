using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;

namespace Atkelar.LegoControlII
{
    /// <summary>
    /// Encapsulates a connection to the Lego Control II interface driver on the Arduino
    /// </summary>
    public class LegoController
        : IDisposable
    {
        private SerialPort _Port;

        /// <summary>
        /// Creates a controller communication object, connecting via the specified COM port.
        /// </summary>
        /// <param name="portName">The (platform specific) name of the port to use, /dev/tty... or COM1:...</param>
        public LegoController(string portName)
        {
            _Port = new SerialPort(portName);
            _Port.BaudRate = 9600;
            _Port.StopBits = StopBits.One;
            _Port.Parity = Parity.None;
            _Port.DataBits = 8;
            _Port.NewLine = "\r\n";
            _Port.Encoding = System.Text.Encoding.UTF8;
            _Port.ReadTimeout = 20000;
            _Port.WriteTimeout = 500;
            _Port.RtsEnable = false;
            _Port.DtrEnable = false;
            _Port.Handshake = Handshake.None;
            _Port.ReceivedBytesThreshold = 1;
            _Port.ErrorReceived += PortErrorHandler;
            _Port.DataReceived += PortIncomingHandler;
        }

        internal void EndMode(OperationalMode modeToEnd)
        {
            if (IsOpen)
            {
                if (Mode == modeToEnd)
                {
                    SendCommandAndHandleGenericResponse("init");
                    Mode = OperationalMode.Init;
                }
            }
        }

        private volatile bool CommandPending = false;

        internal event EventHandler<ServoFinishedEventArgs> ServoFinished;

        internal class ReplyReceivedEventArgs
            : EventArgs
        {
            public bool Handled { get; set; }
            public string ReplyText { get; set; }
        }
        internal event EventHandler<ReplyReceivedEventArgs> IncomingLine;
        private Queue<string> _ReceivedLines = new Queue<string>();   // should be empty, as communication is normally handled via event handler and interrupt handler.

        private void PortIncomingHandler(object sender, SerialDataReceivedEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine(e.EventType);
            if (sender == _Port && _Port.BytesToRead > 0)
            {
                // we only ever get "lines" of text, it is save to wait for the completion of the next line in case data arrived.
                string line = _Port.ReadLine().Trim();
                //Console.WriteLine("Received: " + line);
                if (line == "rdy")
                {
                    //Console.WriteLine("RDY!");
                    CommandPending = false;
                }
                else
                {
                    if (line.StartsWith("err-"))
                    {
                        LastError = line;
                    }
                    else
                    {
                        if (line.StartsWith("trap:"))
                        {
                            int channel = int.Parse(line.Substring(5).Trim());
                            if (ServoFinished != null)
                                ServoFinished(this, new ServoFinishedEventArgs(channel));
                        }
                        else
                        {
                            bool handled = false;
                            if (IncomingLine != null)
                            {
                                var e1 = new ReplyReceivedEventArgs() { ReplyText = line, Handled = false };
                                IncomingLine(this, e1);
                                handled = e1.Handled;
                            }
                            if (!handled)
                                _ReceivedLines.Enqueue(line);
                        }
                    }
                }
            }
        }

        private void PortErrorHandler(object sender, SerialErrorReceivedEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine(e.EventType);
            LastError = "COM ERROR!";
        }

        string _Version;
        private MotorChannelInfo[] MotorChannels;

        /// <summary>
        /// Gets the current "mode" the controller is in.
        /// </summary>
        public OperationalMode Mode { get; private set; } = OperationalMode.Disconnected;

        private void ValidateState(OperationalMode requiredMode = OperationalMode.All, bool requireOpen = true)
        {
            if (_Port == null)
                throw new ObjectDisposedException(nameof(LegoController));
            if (requireOpen && !IsOpen)
                throw new InvalidOperationException("The controller object is not connected! Call Open() first!");
            if ((Mode & requiredMode) == 0)
                throw new InvalidOperationException(string.Format("The controller object is not in a valid state! Expected {0}, got {1}!", requiredMode, Mode));
        }

        /// <summary>
        /// Sets the motor channels for the free or auto modes.
        /// </summary>
        /// <param name="motorData">The channel mapping info for motor channel 0,1 and 2 or less, in order.</param>
        public void SetupMotor(params MotorChannelInfo[] motorData)
        {
            if (motorData.Length > 3)
                throw new ArgumentOutOfRangeException(nameof(motorData), "There are only up to three motor channels!");
            ValidateState(OperationalMode.Init);
            if (motorData.Length == 0)
            {
                SendCommandAndHandleGenericResponse("reset");
                MotorChannels = new MotorChannelInfo[0];
            }
            else
            {
                MotorChannels = new MotorChannelInfo[motorData.Length];
                StringBuilder sb = new StringBuilder();
                sb.Append("mot");
                for (int i = 0; i < motorData.Length; i++)
                {
                    sb.AppendFormat(":{0}:{1}:{2}", motorData[i].Channel, motorData[i].GetCalibrationString(), motorData[i].MappedInput.HasValue ? motorData[i].MappedInput.ToString() : "-");
                    MotorChannels[i] = motorData[i];
                }
                SendCommandAndHandleGenericResponse(sb.ToString());
            }
        }

        internal bool MotorHasCounter(int motorChannel)
        {
            if (motorChannel >= MotorChannels.Length)
                throw new ArgumentOutOfRangeException(nameof(motorChannel), motorChannel, "Not available!");
            return MotorChannels[motorChannel].MappedInput.HasValue;
        }

        internal void WaitForCommandComplete(int timeout = 120)
        {
            //Console.Write("...");
            var timeoutAt = DateTime.Now.AddSeconds(timeout);
            while (CommandPending)
            {
                System.Threading.Thread.Sleep(10);
                ThrowOnError();
                if (DateTime.Now > timeoutAt)
                    throw new InvalidOperationException("Timeout in wait for completion code of driver!");
                //Console.Write(".");
            }
        }

        private void ThrowOnError()
        {
            var l = LastError;
            LastError = null;
            if (l != null)
            {
                throw new InvalidOperationException($"Error in driver: {l}");
            }
        }

        /// <summary>
        /// Sends a command, reacts to the returned information and expects a specific prefix before the next prompt.
        /// </summary>
        /// <param name="command">The command to execute.</param>
        /// <param name="responsePrefix">The expected prefix of the respons.</param>
        /// <returns>The response, if it arrived, as defined by the prefix.</returns>
        internal string SendCommandAndReadExectedResponse(string command, string responsePrefix)
        {
            string response = null;
            EventHandler<ReplyReceivedEventArgs> handler =
                (s, e) =>
                {
                    if (e.ReplyText.StartsWith(responsePrefix) && response == null)
                    {
                        e.Handled = true;
                        response = e.ReplyText;
                    };
                };
            try
            {
                this.IncomingLine += handler;
                SendCommand(command);
                WaitForCommandComplete();
            }
            finally
            {
                this.IncomingLine -= handler;
            }

            ThrowOnError();

            return response;
        }

        /// <summary>
        /// Runs a command and waits for a simple "ok" i.e. the next prompt from the controller.
        /// </summary>
        /// <param name="commandtext">The command text to send.</param>
        internal void SendCommandAndHandleGenericResponse(string commandtext)
        {
            SendCommand(commandtext);
            WaitForCommandComplete();
            ThrowOnError();
        }

        class CalData
        {
            public int MinValue;
            public List<int> CalLocations = new List<int>();
            public List<int> CalPoints = new List<int>();

            public double[] Corrections;

            internal void Calibrate(int absMin, int absMax)
            {

                Corrections = new double[CalPoints.Count];

                for (int i = 0; i < Corrections.Length; i++)
                {
                    double targetValue = Interpolate(absMin, absMax, CalLocations[i]);
                    Corrections[i] = (targetValue / CalPoints[i]);
                    //Console.WriteLine("#{0}|{4} - {1}->{3:0.00} @ {2:0.00}%", i, CalPoints[i], Corrections[i], targetValue, CalLocations[i]);
                }
            }

            private double Interpolate(int absMin, int absMax, int stepIndex)
            {
                stepIndex -= 1;   // zero base, now 0..254
                double h = absMax - absMin;
                double k = h / 254;
                return k * stepIndex + absMin;
            }

            internal string CalibrationString()
            {
                StringBuilder sb = new StringBuilder();
                sb.Append(MinValue.ToString("x2"));
                for (int i = 0; i < Corrections.Length; i++)
                {
                    sb.Append('~');
                    sb.Append(((byte)(CalLocations[i] * Corrections[i])).ToString("x2"));
                }
                return sb.ToString();
            }
        }

        /// <summary>
        /// Runs the motor calibration code and returns the computd calibration string.
        /// </summary>
        /// <param name="channel">The motor channel on the Lego controller to run with. The sensor module has to be connected to channel 6 for this!</param>
        /// <returns>The string to use for the mapping of motors.</returns>
        public string GetMotorCalibration(MotorChannel channel)
        {
            _ReceivedLines.Clear();
            string s;
            CalData[] cal = new CalData[2];
            cal[0] = new CalData();
            cal[1] = new CalData();
            bool completed = false;
            SendCommand(string.Format("cal:{0}", channel));
            EventHandler<ReplyReceivedEventArgs> handler =
            (s2, e) =>
            {
                if (e.ReplyText.StartsWith(string.Format("cal:", channel)))
                {
                    e.Handled = true;
                    s = e.ReplyText.Substring(6);
                    if (s.StartsWith("fail") || s.StartsWith("err-"))
                    {
                        LastError = s;
                        ThrowOnError();
                    }
                    if (s.StartsWith("min:"))
                    {
                        string[] temp = s.Substring(4).Split(':');
                        if (temp.Length != 2)
                            throw new InvalidOperationException("Min line invlaid");

                        cal[0].MinValue = int.Parse(temp[0]);
                        cal[1].MinValue = int.Parse(temp[1]);
                    }
                    if (s == "end")
                    {
                        // need to find the HIGHEST min and LOWEST max value for value distribution...
                        int absMin = Math.Max(cal[0].CalPoints[0], cal[1].CalPoints[0]);
                        int absMax = Math.Min(cal[0].CalPoints[cal[0].CalPoints.Count - 1], cal[1].CalPoints[cal[1].CalPoints.Count - 1]);

                        //Console.WriteLine("min/max: {0}/{1}", absMin, absMax);

                        cal[0].Calibrate(absMin, absMax);
                        cal[1].Calibrate(absMin, absMax);
                        completed = true;
                    }
                    if (s.StartsWith("dat:"))
                    {
                        var temp = s.Substring(4).Split(':');
                        if (temp.Length != 3)
                            throw new InvalidOperationException("invlaid data line!");

                        int calIdx = int.Parse(temp[0]);

                        cal[0].CalLocations.Add(calIdx);
                        cal[1].CalLocations.Add(calIdx);

                        cal[0].CalPoints.Add(int.Parse(temp[1]));
                        cal[1].CalPoints.Add(int.Parse(temp[2]));
                    }
                }
            };
            try
            {
                this.IncomingLine += handler;
                WaitForCommandComplete(300);
            }
            finally
            {
                this.IncomingLine -= handler;
            }
            if (completed)
                return cal[0].CalibrationString() + "~" + cal[1].CalibrationString();
            return null;
        }

        /// <summary>
        /// Connect to the driver module and query the version information during reset.
        /// </summary>
        public void Open()
        {
            ValidateState(requireOpen: false);
            if (IsOpen)
                throw new InvalidOperationException("The controller object is connected! Call Close() first!");
            _Port.Open();
            _ReceivedLines.Clear();
            SendCommand("reset");
            WaitForCommandComplete();

            string line;
            bool banner = false, version = false;
            while (_ReceivedLines.Count > 0)
            {
                line = _ReceivedLines.Dequeue();
                System.Diagnostics.Debug.WriteLine(line);
                if (line.StartsWith("LEGO Control-II Driver"))  // banner message!
                {
                    banner = true;
                }
                if (line.StartsWith("Version: "))
                {
                    _Version = line.Substring(9).Trim();
                    version = true;
                }
            }
            ThrowOnError();
            if (banner && version)
            {
                Mode = OperationalMode.Init;
            }
            else
            {
                throw new InvalidOperationException("Initialization failed!");
            }
            // SendCommand("free");
            // SendCommand("set:+-+-+-");
        }

        /// <summary>
        /// Starts the "free" mode in the driver and wraps around a "free mode controller" object
        /// for easy access to the features there.
        /// </summary>
        /// <returns>The open free mode.</returns>
        public FreeModeController OpenFree()
        {
            ValidateState();
            if (Mode != OperationalMode.Init)
                throw new InvalidOperationException("The controller is not in the 'init' state!");
            var x = new FreeModeController(this);
            SendCommandAndHandleGenericResponse("free");
            Mode = OperationalMode.Free;
            return x;
        }

        /// <summary>
        /// Gets the version number string as reported by the driver during the connection.
        /// </summary>
        public string ConnectedVersion
        {
            get => _Version;
        }

        internal void SendCommand(string commandtext)
        {
            if (CommandPending)
                WaitForCommandComplete();
            CommandPending = true;
            //System.Diagnostics.Debug.WriteLine("sending command: " + commandtext);
            //Console.WriteLine("sending command: " + commandtext);
            _Port.WriteLine(commandtext);
        }

        /// <summary>
        /// Close the connection with the driver.
        /// </summary>
        public void Close()
        {
            ValidateState();
            this.SendCommandAndHandleGenericResponse("reset");
            _Port.Close();
            _Version = null;
            Mode = OperationalMode.Disconnected;
        }

        /// <summary>
        /// True if the class is currently connected to the driver code.
        /// </summary>
        public bool IsOpen
        {
            get => _Port.IsOpen;
        }


        /// <summary>
        /// Gets the last received error message from the driver (i.e. the most recent 'err-*' line)
        /// </summary>
        public string LastError { get; private set; }

        /// <summary>
        /// Gets the number of configured motor channels.
        /// </summary>
        public byte ConfiguredMotorChannels { get => (byte)(MotorChannels?.Length).GetValueOrDefault(); }

        public void Dispose()
        {
            if (_Port != null)
                _Port.Dispose();
            _Port = null;
            _Version = null;
        }

        /// <summary>
        /// Use this calibration data as a starting point if you don't want to go with the full calibration routine.
        /// </summary>
        public const string DefaultMediumSpeedCalibration = "7F~01~18~4e~7e~ad~db~ff~7F~01~18~4e~7e~ad~db~ff";
    }
}