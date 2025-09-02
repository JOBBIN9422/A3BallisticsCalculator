using MathNet.Numerics;
using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace A3BallisticsWPF
{
    public class Calculator
    {
        public static readonly double GRAVITY = 9.8066;

        //return the angle of departure for the target zero distance, initial velocity, and drag
        public static double CalcVanillaZeroAngle(double zeroRange, double muzzleVelocity, double airFriction, double maxDeltaT)
        {
            double time = 0.0;

            Vector<double> currShotPos = Vector<double>.Build.Dense(3);
            Vector<double> currSpeed = Vector<double>.Build.Dense(new double[] { muzzleVelocity, 0, 0 });

            while (time < 8.0)
            {
                double distLeft = zeroRange - currShotPos.At(0);
                double traveled = currSpeed.At(0) * maxDeltaT;

                if (distLeft < traveled)
                {
                    double deltaT = distLeft / currSpeed.At(0);
                    currSpeed = currSpeed - Vector<double>.Build.Dense(new double[] { 0, GRAVITY * deltaT, 0 });
                    currShotPos = currShotPos + (currSpeed * deltaT);
                    time += deltaT;

                    break;
                }
                else
                {
                    double deltaT = maxDeltaT;
                    currShotPos = currShotPos + (currSpeed * deltaT);
                    time += deltaT;
                    currSpeed = currSpeed + currSpeed * (currSpeed.L2Norm() * airFriction * deltaT);
                    currSpeed = currSpeed - Vector<double>.Build.Dense(new double[] { 0, GRAVITY * deltaT, 0 });
                }
            }

            return 180 / Math.PI * -Math.Atan(currShotPos.At(1) / zeroRange) * 60;
        }

        public static float CalcVanillaZeroAngleFloat(float zeroRange, float muzzleVelocity, float airFriction, float maxDeltaT)
        {
            float time = 0.0f;

            Vector<float> currShotPos = Vector<float>.Build.Dense(3);
            Vector<float> currSpeed = Vector<float>.Build.Dense(new float[] { muzzleVelocity, 0, 0 });

            while (time < 8.0)
            {
                float distLeft = zeroRange - currShotPos.At(0);
                float traveled = currSpeed.At(0) * maxDeltaT;

                if (distLeft < traveled)
                {
                    float deltaT = distLeft / currSpeed.At(0);
                    currSpeed = currSpeed - Vector<float>.Build.Dense(new float[] { 0, (float)(GRAVITY * deltaT), 0 });
                    currShotPos = currShotPos + (currSpeed * deltaT);
                    time += deltaT;

                    break;
                }
                else
                {
                    float deltaT = maxDeltaT;
                    currShotPos = currShotPos + (currSpeed * deltaT);
                    time += deltaT;
                    currSpeed = currSpeed + currSpeed * ((float)currSpeed.L2Norm() * airFriction * deltaT);
                    currSpeed = currSpeed - Vector<float>.Build.Dense(new float[] { 0, (float)(GRAVITY * deltaT), 0 });
                }
            }

            return (float)(180 / Math.PI * -Math.Atan(currShotPos.At(1) / zeroRange) * 60);
        }

        public static SimulationRecord BuildRangeTable(double launchAngle, double muzzleVelocity, double airFriction, double deltaT, double cutoffRange)
        {
            //convert from MOA to rads
            double launchAngleRads = launchAngle / 60 * (Math.PI / 180);
            double time = 0.0;

            //TODO: account for bore height (init y of currShotPos)
            //shot xy velocity and position on each step
            //Vector<double> currShotPos = Vector<double>.Build.Dense(3);
            Vector<double> currShotPos = Vector<double>.Build.Dense(new double[] { 0, 0, 0 });
            Vector<double> currSpeed = Vector<double>.Build.Dense(new double[] { muzzleVelocity * Math.Cos(launchAngleRads), muzzleVelocity * Math.Sin(launchAngleRads), 0 });

            SimulationRecord record = new SimulationRecord();
            record.LaunchAngleMOA = launchAngle;

            while (currShotPos.At(0) < cutoffRange)
            {
                //log current time
                record.Timestamps.Add(time);

                //log current position
                record.XPositions.Add(currShotPos.At(0));
                record.YPositions.Add(currShotPos.At(1));

                //log current velocity 
                record.XVelocities.Add(currSpeed.At(0));
                record.YVelocities.Add(currSpeed.At(1));
                record.AbsVelocities.Add(currSpeed.L2Norm());

                //update shot pos
                currShotPos = currShotPos + (currSpeed * deltaT);

                //update velocity
                currSpeed = currSpeed + currSpeed * (currSpeed.L2Norm() * airFriction * deltaT);
                currSpeed = currSpeed - Vector<double>.Build.Dense(new double[] { 0, GRAVITY * deltaT, 0 });

                //update time
                time += deltaT;
            }

            //if last xVal is < cutoffRange, we need to interpolate to cutoff range from most recent shot position
            if (record.XPositions[record.XPositions.Count - 1] < cutoffRange)
            {
                //add last "step" to each list
                record.Timestamps.Add(time);

                record.XPositions.Add(currShotPos.At(0));
                record.YPositions.Add(currShotPos.At(1));

                record.XVelocities.Add(currSpeed.At(0));
                record.YVelocities.Add(currSpeed.At(1));
                record.AbsVelocities.Add(currSpeed.L2Norm());

                //cubic spline interpolation for x(t), y(t), v(t), v_x(t), and v_y(t)
                record.CalcInterpolations();

                //iterate over the last time step in small increments to find the time where the bullet reaches the requested range
                double stopTime = record.SolveForTimeAtX(cutoffRange, time - deltaT);
                Debug.WriteLine($"Found stop values: t = {stopTime}, x(t) = {record.XPosOverTime.Interpolate(stopTime)}, v(t) = {record.AbsVelocityOverTime.Interpolate(stopTime)}, v_x(t) = {record.XVelocityOverTime.Interpolate(stopTime)}, v_y(t) = {record.YVelocityOverTime.Interpolate(stopTime)}");

                //replace last entries based on interpolated time
                record.Timestamps[record.Timestamps.Count - 1] = stopTime;
                record.XPositions[record.XPositions.Count - 1] = record.XPosOverTime.Interpolate(stopTime);
                record.YPositions[record.YPositions.Count - 1] = record.YPosOverTime.Interpolate(stopTime);
                record.AbsVelocities[record.AbsVelocities.Count - 1] = record.AbsVelocityOverTime.Interpolate(stopTime);
                record.XVelocities[record.XVelocities.Count - 1] = record.XVelocityOverTime.Interpolate(stopTime);
                record.YVelocities[record.YVelocities.Count - 1] = record.YVelocityOverTime.Interpolate(stopTime);
            }
            else
            {
                record.CalcInterpolations();
            }

            return record;
        }
    }
}
