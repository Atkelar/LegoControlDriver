using System;
using Atkelar.LegoControlII;

namespace Atkelar.LegoPlotter
{
    public class AlignmentController
        : IDisposable
    {
        private FreeModeController _Controller;

        internal AlignmentController(FreeModeController plotterController)
        {
            this._Controller = plotterController;
        }

        public void Dispose()
        {
            _Controller.FullStop();
        }

        public void Left()
        {
            _Controller.ReverseOrStop(1, Direction.Forward);
        }
        public void Right()
        {
            _Controller.ReverseOrStop(1, Direction.Reverse);
        }

        internal void FullStop()
        {
            _Controller.FullStop();
        }

        internal void Up()
        {
            _Controller.ReverseOrStop(0, Direction.Forward);
        }

        internal void Down()
        {
            _Controller.ReverseOrStop(0, Direction.Reverse);
        }

        internal void PenDown()
        {
            _Controller.ReverseOrStop(2, Direction.Forward);
        }

        internal void PenUp()
        {
            _Controller.ReverseOrStop(2, Direction.Reverse);
        }
    }
}