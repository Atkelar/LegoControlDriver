using System;
using Atkelar.LegoControlII;

namespace Atkelar.LegoPlotter
{
    internal class PlotterController : IDisposable
    {
        private FreeModeController _Control;

        public PlotterController(string portname, string calibrationA, string calibrationB, string calibrationC)
        {
            _BaseController = new LegoController(portname);

            Calibration = new string[3];
            Calibration[0] = calibrationA;
            Calibration[1] = calibrationB;
            Calibration[2] = calibrationC;
        }

        public void Open()
        {
            _BaseController.Open();
            _BaseController.SetupMotor(
                    new MotorChannelInfo(MotorChannel.A, Calibration[0], 6),
                    new MotorChannelInfo(MotorChannel.B, Calibration[1], 7),
                    new MotorChannelInfo(MotorChannel.C, Calibration[2], null)
                );
            _Control = _BaseController.OpenFree();
        }

        public AlignmentController OpenAlign()
        {
            if (_Control == null)
                throw new InvalidOperationException("Plotter controller is not in 'open' state!");
            return new AlignmentController(this._Control);
        }

        private int _X = 0;
        private int _Y = 0;

        private bool _IsPenUp;
        private int xBacklashDown;
        private int yBacklashDown;
        private int xBacklashUp;
        private int yBacklashUp;
        private Direction _LastXDir;
        private Direction _LastYDir;
        private LegoController _BaseController;
        private string[] Calibration;

        public void Dispose()
        {
            var c = _Control;
            _Control = null;
            if (c != null)
            {
                if (PenUpOnDispose)
                    PenUp();
                c.FullStop();
                c.Dispose();
            }
            var b = _BaseController;
            _BaseController = null;
            if (b != null)
            {
                b.Dispose();
            }
        }

        /// <summary>
        /// Puts the pen down, if it isn't already so.
        /// </summary>
        public void PenDown()
        {
            if (_IsPenUp)
            {
                _Control.Motor(2, Direction.Reverse);
                System.Threading.Thread.Sleep(480); // down is slightly shorter to even out and prevent "binding" conditions.
                _Control.Motor(2);
                System.Threading.Thread.Sleep(100);
            }
            _IsPenUp = false;
        }

        /// <summary>
        /// Puts the pen up, if it isn't already so.
        /// </summary>
        public void PenUp()
        {
            if (!_IsPenUp)
            {
                _Control.Motor(2, Direction.Forward);
                System.Threading.Thread.Sleep(500);
                _Control.Motor(2);
                System.Threading.Thread.Sleep(100);
            }
            _IsPenUp = true;
        }

        /// <summary>
        /// Returns the pen to the home position.
        /// </summary>

        public void Home()
        {
            PenUp();
            MoveTo(0, 0);
        }

        /// <summary>
        /// Set to true by default; indicates whether dispose should first run a pen-up command.
        /// </summary>
        public bool PenUpOnDispose { get; set; }

        /// <summary>
        /// Size in X-direction, in mm.
        /// </summary>
        public const float Width = xLimit * UnitsToMil;
        /// <summary>
        /// Size in Y-direction, in mm.
        /// </summary>
        public const float Height = yLimit * UnitsToMil;

        private const int xLimit = 246;
        private const int yLimit = 282;
        private const float UnitsToMil = 0.535f;

        /// <summary>
        /// Move the pen to position x/y, specified in mm.
        /// </summary>
        /// <param name="x">X-coordinate.</param>
        /// <param name="y">Y-coordinate.</param>
        public void MoveTo(float x, float y)
        {
            x /= UnitsToMil;
            y /= UnitsToMil;
            // now we need ot interpolate from the current position to the new requested position...
            int dx = (int)Math.Round(x - _X);
            int dy = (int)Math.Round(y - _Y);

            // Limit movement to bounding rectangle.
            if (_X + dx < 0)
                dx = -_X;
            if (_Y + dy < 0)
                dy = -_Y;
            if (_X + dx > xLimit)
                dx = xLimit - _X;
            if (_Y + dy > yLimit)
                dx = yLimit - _Y;


            if (dx == 0 && dy == 0) // already there!
                return;

            //Console.WriteLine("move {0}/{1} to {2}/{3} from {4}/{5}", dx, dy, x, y, _X, _Y);

            Direction xDir = DirectionFromDelta(dx * -1);   // x-axis is flipped...
            Direction yDir = DirectionFromDelta(dy);

            // backlash correction on direction change; 
            // use different values for "pen down" and "pen up" versions...
            bool correction = false;
            int blX = _IsPenUp ? xBacklashUp : xBacklashDown;
            int blY = _IsPenUp ? yBacklashUp : yBacklashDown;

            if (_LastXDir != Direction.Stop && _LastXDir != xDir && xDir != Direction.Stop && blX > 0)
            {
                // turnaround
                correction = true;
                _Control.ServoMotor(1, xDir, blX, 100);
            }
            if (_LastYDir != Direction.Stop && _LastYDir != yDir && yDir != Direction.Stop && blY > 0)
            {
                // turnaround
                correction = true;
                _Control.ServoMotor(0, yDir, blY, 100);
            }

            if (correction)
                _Control.WaitForServoFinished(0, 1);


            byte xSpeed = 100;
            byte ySpeed = 100;

            int nx = Math.Abs(dx);
            int ny = Math.Abs(dy);

            if (!_IsPenUp)  // speed adjustment only when pen is down...
            {
                if (dx != dy && dx != 0 && dy != 0)
                {
                    // we need to move diagonally of some odd angle - i.e. not 0, 90 or 45°
                    if (nx > ny)
                    {
                        // more across than up...
                        ySpeed = (byte)(100 * ny / nx);
                        if (ySpeed < 1)
                            ySpeed = 1;
                        if (ySpeed > 100)
                            ySpeed = 100;
                    }
                    else
                    {
                        xSpeed = (byte)(100 * nx / ny);
                        if (xSpeed < 1)
                            xSpeed = 1;
                        if (xSpeed > 100)
                            xSpeed = 100;
                    }
                }
            }

            //Console.WriteLine("-> {0}/{1} @ {2}/{3}", nx, ny, xSpeed, ySpeed);
            // if (xDir == Direction.Stop || yDir == Direction.Stop)
            // {
            //     Console.WriteLine("x, 1= {0} by {1}", xDir, nx);
            //     Console.WriteLine("y, 0= {0} by {1}", yDir, ny);
            // }
            _Control.ServoMotor(1, xDir, nx, xSpeed);
            _Control.ServoMotor(0, yDir, ny, ySpeed);
            _Control.WaitForServoFinished(0, 1);

            _X += dx;
            _Y += dy;
            if (!_IsPenUp)
            {
                if (xDir != Direction.Stop)
                    _LastXDir = xDir;
                if (yDir != Direction.Stop)
                    _LastYDir = yDir;
            }
        }

        /// <summary>
        /// Close the plotter interface and clean up.
        /// </summary>
        public void Close()
        {
            var c = _Control;
            _Control = null;
            if (c != null)
                c.Dispose();
            if (_BaseController != null)
                _BaseController.Close();
        }

        /// <summary>
        /// Run the X-side motor calibration (channel B)
        /// </summary>
        /// <returns>The calibration string.</returns>
        public string RunMotorCalibrationX()
        {
            _BaseController.Open();
            string s = _BaseController.GetMotorCalibration(MotorChannel.B);
            _BaseController.Close();
            return s;
        }

        /// <summary>
        /// Run the Y-side motor calibration (channel A)
        /// </summary>
        /// <returns>The calibration string.</returns>
        internal string RunMotorCalibrationY()
        {
            _BaseController.Open();
            string s = _BaseController.GetMotorCalibration(MotorChannel.A);
            _BaseController.Close();
            return s;
        }

        /// <summary>
        /// Marks the current position as "home".
        /// </summary>
        /// <param name="assumePenUp">True to assume that the pen is up. This method will also "up" the pen if needed.</param>
        internal void SetAligned(bool assumePenUp)
        {
            _Control.FullStop();
            _X = 0;
            _Y = 0;
            _IsPenUp = assumePenUp;
            PenUp();
            PenUpOnDispose = true;  // we have home position here, from now on we "up" the pen when done.
        }

        private Direction DirectionFromDelta(int d)
        {
            if (d == 0)
                return Direction.Stop;
            if (d < 0)
                return Direction.Reverse;
            else
                return Direction.Forward;
        }

        /// <summary>
        /// Unused currently.
        /// </summary>
        /// <param name="xDown"></param>
        /// <param name="yDown"></param>
        /// <param name="xUp"></param>
        /// <param name="yUp"></param>
        [Obsolete("Doesn't work well... needs work.")]
        public void SetBacklashCalibration(float xDown, float yDown, float xUp, float yUp)
        {
            xBacklashDown = (int)Math.Round(Math.Abs(xDown / UnitsToMil));
            yBacklashDown = (int)Math.Round(Math.Abs(yDown / UnitsToMil));
            xBacklashUp = (int)Math.Round(Math.Abs(xUp / UnitsToMil));
            yBacklashUp = (int)Math.Round(Math.Abs(yUp / UnitsToMil));
        }

        /// <summary>
        /// Prints a set of markers with different backlash compenstation values to pick the "best" match.
        /// </summary>
        public void BacklashCalibrationSheet()
        {

            // trying different pattern to realize how much backlash x/y for pen up/down is required...

            xBacklashDown = 0;  // reset calibration to make sure we capture RAW values
            yBacklashDown = 0;
            xBacklashUp = 0;
            yBacklashUp = 0;

            PenUp();

            for (int i = 0; i < 5; i++)
            {
                float y = i * 20 + 5;
                MoveTo(5, y);
                PenDown();
                MoveTo(15, y);
                MoveTo(15, y + 10);
                MoveTo(25, y + 10); // outer edge...
                MoveTo(15 - i, y + 10); // check for X-deflection...
                MoveTo(15 - i, y - i);
                MoveTo(5, y - i);
                PenUp();
            }


            for (int i = 0; i < 5; i++)
            {
                float y = i * 20 + 5;
                MoveTo(35, y);
                MoveTo(45, y + 5);
                PenDown();
                PenUp();
                MoveTo(50, y + 10);
                MoveTo(45 - i, y + 5 - i);
                PenDown();
                PenUp();
            }
        }
    }
}