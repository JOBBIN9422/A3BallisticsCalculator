using CsvHelper;
using MathNet.Numerics;
using MathNet.Numerics.Interpolation;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace A3BallisticsWPF
{
    public class SimulationRecord
    {
        //the projectile's angle of departure
        public double LaunchAngleMOA { get; set; }

        //parallel lists for: time, X, Y, Holdovers/unders in MOA, Vx, Vy, and |V|
        public List<double> Timestamps { get; set; } = new List<double>();
        public List<double> XPositions { get; set; } = new List<double>();
        public List<double> YPositions { get; set; } = new List<double>();
        public List<double> XVelocities { get; set; } = new List<double>();
        public List<double> YVelocities { get; set; } = new List<double>();
        public List<double> AbsVelocities { get; set; } = new List<double>();

        //cubic spline interpolations for position/speed over time on each axis
        public IInterpolation XPosOverTime { get; set; }
        public IInterpolation YPosOverTime { get; set; }
        public IInterpolation AbsVelocityOverTime { get; set; }
        public IInterpolation XVelocityOverTime { get; set; }
        public IInterpolation YVelocityOverTime { get; set; }

        public double SolveForTimeAtX(double x, double startTime)
        {
            if (Timestamps.Count < 2)
            {
                throw new Exception("Not enough data points.");
            }

            double deltaT = Timestamps[1] - Timestamps[0];
            double minError = double.MaxValue;
            double targetTime = 0.0;

            //iterate over the last time step in small increments to find the time where the bullet reaches the requested range 
            for (double t = startTime; t < Timestamps.Last(); t += deltaT / 100.0)
            {
                double interpolatedPos = XPosOverTime.Interpolate(t);
                double currDiff = Math.Abs(interpolatedPos - x);

                //choose the t with the smallest error
                if (currDiff < minError)
                {
                    targetTime = t;
                    minError = currDiff;
                }
            }

            return targetTime;
        }

        //build a new SimulationRecord from the existing one with data sampled at set distances instead of set times
        public SimulationRecord ReInterpolate(double distanceStepSize)
        {
            //use same interpolated curves from existing record
            SimulationRecord newRecord = new SimulationRecord
            {
                LaunchAngleMOA = this.LaunchAngleMOA,
                XPosOverTime = this.XPosOverTime,
                YPosOverTime = this.YPosOverTime,
                AbsVelocityOverTime = this.AbsVelocityOverTime,
                XVelocityOverTime = this.AbsVelocityOverTime,
                YVelocityOverTime = this.YVelocityOverTime
            };

            double distTraveled = 0.0;
            double currStepTime = 0.0;

            //walk in increments of stepSize until we've covered the whole flight path
            do
            {
                currStepTime = SolveForTimeAtX(distTraveled, currStepTime);
                newRecord.Timestamps.Add(currStepTime);
                newRecord.XPositions.Add(distTraveled);
                newRecord.YPositions.Add(YPosOverTime.Interpolate(currStepTime));
                newRecord.XVelocities.Add(XVelocityOverTime.Interpolate(currStepTime));
                newRecord.YVelocities.Add(YVelocityOverTime.Interpolate(currStepTime));
                newRecord.AbsVelocities.Add(AbsVelocityOverTime.Interpolate(currStepTime));

                distTraveled += distanceStepSize;
            } while (Math.Abs(newRecord.XPositions.Last() - XPositions.Last()) >= 1.0);

            Debug.WriteLine(newRecord.XPositions.Last());
            return newRecord;
        }

        public void ExportToCSV(string path, double distanceStepSize)
        {
            SimulationRecord newRecord = ReInterpolate(distanceStepSize);

            //walk in increments of stepSize until we've covered the whole flight path
            using (var writer = new StreamWriter(path))
            {
                writer.WriteLine($"Distance (m),Elevation (MOA),Elevation (m),Time (s),Velocity (m/s)");
                for (int i = 0; i < newRecord.Timestamps.Count; i++)
                {
                    //set holdover to 0 MOA on first pass to prevent NaN (divide by zero err if x[i] == 0)
                    double holdoverMOA = XPositions[i] == 0 ? 0 : Math.Atan(YPositions[i] / XPositions[i]) * 180 / Math.PI * 60;
                    writer.WriteLine($"{XPositions[i]},{holdoverMOA},{YPositions[i]},{Timestamps[i]},{AbsVelocities[i]}");
                }
            }
        }

        public void CalcInterpolations()
        {
            XPosOverTime = Interpolate.CubicSpline(Timestamps, XPositions);
            YPosOverTime = Interpolate.CubicSpline(Timestamps, YPositions);
            AbsVelocityOverTime = Interpolate.CubicSpline(Timestamps, AbsVelocities);
            XVelocityOverTime = Interpolate.CubicSpline(Timestamps, XVelocities);
            YVelocityOverTime = Interpolate.CubicSpline(Timestamps, YVelocities);
        }
    }
}
