using System;
using System.Collections.Generic;
using System.IO;

namespace Atkelar.LegoPlotter
{
    class Program
    {
        static int Syntax()
        {
            Console.WriteLine("Call: alp-control <portname> [mode [filename]]");
            Console.WriteLine();
            Console.WriteLine("  portname:  the USB/Serial port name, e.g. /dev/ttyACM0 or COM1");
            Console.WriteLine("  mode:      plot, interactive, align, cal or backlash");
            Console.WriteLine("  filename:  the plt filename");

            Console.WriteLine("cal: Run motor channel calibration on A and B channel.");
            Console.WriteLine("     Will create the file 'alp-cal.txt' in the current directory.");
            Console.WriteLine("backlash: print out backlash calibration sheet.");
            Console.WriteLine("interactive: run the plotter in interactive mode, allowing direct cursor key movement.");
            Console.WriteLine("align: run in interactive mode to set the 'home' position. The 'plot' mode assumes this has been done!");
            Console.WriteLine("plot: reads FILENAME and sends the plot commands to output the drawing.");

            Console.WriteLine("The sequence of operations should be: cal -> align -> plot at a minimum!");
            Console.WriteLine("Before running any of the actual modes, run cal and align, since all others requre the calibration data AND an initial 'home' position at startup!");

            return 1;
        }

        static bool RunFreeMode(PlotterController plt)
        {
            Console.WriteLine(FreeModeControls);
            using (var control2 = plt.OpenAlign())
            {
                while (true)
                {
                    var key = Console.ReadKey();
                    switch (key.Key)
                    {
                        case ConsoleKey.Spacebar:
                            control2.FullStop();
                            break;
                        case ConsoleKey.LeftArrow:
                            control2.Left();
                            break;
                        case ConsoleKey.RightArrow:
                            control2.Right();
                            break;
                        case ConsoleKey.UpArrow:
                            control2.Up();
                            break;
                        case ConsoleKey.DownArrow:
                            control2.Down();
                            break;
                        case ConsoleKey.PageUp:
                            control2.PenDown();
                            break;
                        case ConsoleKey.PageDown:
                            control2.PenUp();
                            break;
                        case ConsoleKey.Enter:
                            return true;
                        case ConsoleKey.Escape:
                            return false;
                    }
                }
            }
        }

        const string FreeModeControls = "  Space=full stop, Cursor=move, pgup/dn=pen.";
        // const string MotorACal = "44~01~1a~4a~77~a5~d7~ff~63~01~18~4e~7f~b4~e6~fb";
        // const string MotorBCal = "3c~01~1a~4b~78~a5~df~ff~4b~01~1f~59~8e~c2~f7~ff";
        // const string MotorACal = "71~01~19~4b~79~af~d7~ff~80~01~1c~4f~90~c3~f3~f7";
        // const string MotorBCal = "55~01~17~47~75~a0~c9~f7~67~01~1b~4c~7b~bb~eb~ff";

        static string MotorACal, MotorBCal;

        static int Main(string[] args)
        {
            Console.WriteLine("Lego Control-II Plotter");
            Console.WriteLine();

            if (args.Length < 1)
                return Syntax();
            bool runInteractive = false;
            bool runAlign = false;
            bool runPlot = false;
            bool runCalibration = false;
            bool runBacklashPage = false;
            string filename = null;
            if (args.Length > 1)
            {
                string mode = args[1].ToLowerInvariant();
                switch (mode)
                {
                    case "cal":
                        runCalibration = true;
                        break;
                    case "plot":
                        if (args.Length < 2)
                        {
                            return Syntax();
                        }
                        filename = args[2];
                        runPlot = true;
                        break;
                    case "align":
                        runAlign = true;
                        break;
                    case "interactive":
                        runInteractive = true;
                        break;
                    case "backlash":
                        runBacklashPage = true;
                        break;
                    default:
                        return Syntax();
                }
            }
            else
            {
                runInteractive = true;
            }


            //plt.SetBacklashCalibration(1, 1, 0.5f, 2);
            if (runCalibration)
            {
                Console.WriteLine("Make sure that the connetions are as follows:");
                Console.WriteLine("Motor A -> Y axis, the moving plate, detector on channel 6.");
                Console.WriteLine("Motor B -> X axis, the pen sled, detector on channel 7.");
                Console.WriteLine("Motor C -> Pen up/down actuator.");

                Console.WriteLine("Calibration mode: put pen in approximate center, hit ENTER to confirm, ESC to cancel.");

                using (var plt = new PlotterController(args[0], LegoControlII.LegoController.DefaultMediumSpeedCalibration, LegoControlII.LegoController.DefaultMediumSpeedCalibration, LegoControlII.LegoController.DefaultMediumSpeedCalibration))
                {
                    plt.Open();
                    if (!RunFreeMode(plt))
                        return 0;
                    plt.Close();
                    Console.WriteLine("Plug detector from motor A (Y-axis) to channel 6...");
                    Console.WriteLine("ENTER when ready.");
                    Console.ReadLine();

                    string y = plt.RunMotorCalibrationY();

                    Console.WriteLine("Plug detector from motor B (X-axis) to channel 6...");
                    Console.WriteLine("ENTER when ready.");
                    Console.ReadLine();
                    string x = plt.RunMotorCalibrationX();
                    Console.WriteLine("Plug detector from motor B (X-axis) to channel 7 and detector from motor A (Y-axis) to channel 6 again!");

                    Console.WriteLine("Motor A calibration: {0}", y);
                    Console.WriteLine("Motor B calibration: {0}", x);

                    using (var output = System.IO.File.CreateText("alp-cal.txt"))
                    {
                        output.WriteLine("# Atkalar LEGO plotter control calibration data, created at {0:yyyy-MM-dd HH:mm:ss}", DateTime.Now);
                        output.WriteLine();
                        output.WriteLine(x);
                        output.WriteLine(y);
                    }
                }

                return 0;
            }

            if (!System.IO.File.Exists("alp-cal.txt"))
            {
                Console.WriteLine("alp-cal.txt is missing, run 'cal' command first!");
                return 4;
            }
            using (var input = System.IO.File.OpenText("alp-cal.txt"))
            {
                string line;
                List<string> calLines = new List<string>();
                while ((line = input.ReadLine()) != null)
                {
                    line = line.Trim();
                    if (line.Length > 0 && !line.StartsWith("#"))
                    {
                        calLines.Add(line);
                        if (calLines.Count >= 2)
                            break;
                    }
                }
                if (calLines.Count < 2)
                    throw new InvalidOperationException("The alp-cal.txt file doesn't have the required two lines of calibratino data!");

                MotorBCal = calLines[0];
                MotorACal = calLines[1];
            }


            using (var plt = new PlotterController(args[0], MotorACal, MotorBCal, LegoControlII.LegoController.DefaultMediumSpeedCalibration))
            {
                plt.Open();
                if (runAlign)    // run "alignment" code.
                {
                    Console.WriteLine("Alignment mode: Put pen into origin and DOWN position, hit ENTER to confirm, ESC to cancel.");

                    if (!RunFreeMode(plt))
                        return 0;
                    // run in "open" interactive mode to align pen up/down

                    plt.SetAligned(false);
                }
                else
                    plt.SetAligned(true);

                if (runInteractive)
                {
                    RunFreeMode(plt);
                }

                if (runBacklashPage)
                {
                    plt.BacklashCalibrationSheet();
                }

                if (runPlot)
                {
                    if (!System.IO.File.Exists(filename))
                    {
                        Console.WriteLine("File {0} not found!", filename);
                        return 2;
                    }
                    HpglParser parser;
                    using (var f = File.OpenText(filename))
                    {
                        parser = HpglParser.Analyze(f);
                    }
                    parser.ScaleToCenter(125, 152);
                    Console.WriteLine("Plotting...");
                    foreach (var item in parser.GetDrawingAsLines())
                    {
                        if (item.Draw)
                            plt.PenDown();
                        else
                            plt.PenUp();
                        plt.MoveTo(item.X, item.Y);
                    }
                    //Console.WriteLine("{0} to {1}/{2}", item.Draw ? "draw" : "move", item.X, item.Y);
                }

                Console.WriteLine("Homing...");
                plt.PenUp();
                plt.MoveTo(0, 0);
            }
            return 0;
        }
    }
}
