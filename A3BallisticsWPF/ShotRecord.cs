using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace A3BallisticsWPF
{
    public class ShotRecord
    {
        public Vector<double> TerminalVelocity { get; set; }
        public Vector<double> TerminalPosition { get; set; }
    }
}
