using System;
using System.Collections.Generic;
using System.IO;

namespace Atkelar.LegoPlotter
{
    internal class HpglParser
    {
        private HpglParser()
        {
        }


        private List<Tuple<string, int>> _Commands = new List<Tuple<string, int>>();
        private List<Command> _ParsedCommands = new List<Command>();

        public class Command
        {
            public string Mnemonic { get; internal set; }
            public string[] Parameters { get; internal set; }
        }

        private static string[] NoArgs = new string[0];
        private static readonly char[] Separators = ", ".ToCharArray();
        float xMin = float.MaxValue, xMax = float.MinValue, yMin = float.MaxValue, yMax = float.MinValue;

        /// <summary>
        /// Reads the input stream as HPGL file and fills the internal parsed command list.
        /// </summary>
        /// <param name="input">The stream to read.</param>
        /// <returns>The parsed and loaded HPGL file.</returns>
        public static HpglParser Analyze(StreamReader input)
        {
            HpglParser load = new HpglParser();
            int lineNumber = 0;
            string line;
            var foundCommands = new List<string>();
            while ((line = input.ReadLine()) != null)
            {
                lineNumber++;
                foreach (var subLine in line.Split(';')) // read all sub-commands...
                {
                    var cmd = subLine.Trim();
                    if (cmd.Length >= 2)
                    {
                        // got a command...
                        string mnemonic = cmd.Substring(0, 2).ToUpper();
                        if (foundCommands.IndexOf(mnemonic) < 0)
                        {
                            foundCommands.Add(mnemonic);
                            load._Commands.Add(new Tuple<string, int>(mnemonic, lineNumber));
                        }
                        string[] pars = NoArgs;
                        if (cmd.Length > 2)
                        {
                            pars = cmd.Substring(2).Trim().Split(Separators, StringSplitOptions.RemoveEmptyEntries);
                            for (int i = 0; i < pars.Length; i++)
                                pars[i] = pars[i].Trim();
                            if (pars.Length == 0)
                                pars = NoArgs;
                        }
                        load._ParsedCommands.Add(new Command() { Mnemonic = mnemonic, Parameters = pars });
                    }
                }
            }

            // foreach (var item in _Commands)
            // {
            //     Console.WriteLine("{0} in line {1}", item.Item1, item.Item2);
            // }

            bool isAbsolute = true;
            float x = 0, y = 0;
            load.xMin = float.MaxValue;
            load.xMax = float.MinValue;
            load.yMin = float.MaxValue;
            load.yMax = float.MinValue;
            foreach (var item in load._ParsedCommands)
            {
                switch (item.Mnemonic)
                {
                    case "IN":
                        isAbsolute = true;
                        break;
                    case "SP":
                        // ignore pen changes...
                        break;
                    case "PD":
                    case "PU":
                        // pen down and pen up commands have coordinate lists.
                        if ((item.Parameters.Length % 2) != 0)
                            throw new InvalidOperationException("PD/PU has odd argumetns!");
                        if (isAbsolute)
                        {
                            // absolute coordinates...
                            for (int i = 0; i < item.Parameters.Length; i += 2)
                            {
                                float x1 = float.Parse(item.Parameters[i]);
                                float y1 = float.Parse(item.Parameters[i + 1]);
                                x = x1;
                                y = y1;
                            }
                        }
                        else
                        {
                            throw new NotImplementedException();
                        }
                        if (x < load.xMin)
                            load.xMin = x;
                        if (x > load.xMax)
                            load.xMax = x;
                        if (y < load.yMin)
                            load.yMin = y;
                        if (y > load.yMax)
                            load.yMax = y;
                        break;
                    default:
                        throw new NotImplementedException($"Unsupported command: {item.Mnemonic} at first line {load._Commands.Find(x => x.Item1 == item.Mnemonic).Item2}...");
                }
            }
            //Console.WriteLine("Extent: {0}/{1} - {2}/{3}", xMin, yMin, xMax, yMax);
            return load;
        }

        private float xScale = 1;
        private float yScale = 1;
        private float xOffset = 0;
        private float yOffset = 0;


        public void ScaleToCenter(float xTarget, float yTarget)
        {
            float w = xMax - xMin;
            float h = yMax - yMin;
            float f1 = xTarget / w;
            float f2 = yTarget / h;

            float f = Math.Min(f1, f2);

            xScale = yScale = f;
            w *= f;
            h *= f;
            xOffset = (xTarget - w) / 2;
            yOffset = (yTarget - h) / 2;

            //Console.WriteLine("Scaling {0}*{1} with {2}/{3} offset", xScale, yScale, xOffset, yOffset);
        }

        public class LineSegment
        {
            public bool Draw { get; internal set; }
            public float X { get; internal set; }
            public float Y { get; internal set; }
        }

        public IEnumerable<LineSegment> GetDrawingAsLines()
        {
            float x = 0, y = 0;
            foreach (var item in _ParsedCommands)
            {
                switch (item.Mnemonic)
                {
                    case "PD":
                    case "PU":
                        if (item.Parameters.Length == 0)
                        {
                            yield return new LineSegment() { X = x, Y = y, Draw = item.Mnemonic == "PD" };
                        }
                        else
                        {
                            for (int i = 0; i < item.Parameters.Length; i += 2)
                            {
                                x = (float.Parse(item.Parameters[i]) - xMin) * xScale + xOffset;
                                y = (float.Parse(item.Parameters[i + 1]) - yMin) * yScale + yOffset;
                                yield return new LineSegment() { X = x, Y = y, Draw = item.Mnemonic == "PD" };
                            }
                        }
                        break;
                }
            }
        }
    }
}