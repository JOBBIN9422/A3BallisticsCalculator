using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace A3BallisticsWPF
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        public SimulationRecord LastRecord { get; private set; } = null;

        private void CalculateButton_Click(object sender, RoutedEventArgs e)
        {
            double muzzleVelocity = double.Parse(MuzzleVelTextBox.Text);
            double zeroRange = double.Parse(ZeroDistTextBox.Text);
            double airFriction = double.Parse(AirFrictionTextBox.Text);
            double deltaT = double.Parse(DeltaTTextBox.Text);
            double stepSize = double.Parse(StepSizeTextBox.Text);
            double maxRange = double.Parse(ChartMaxRangeTextBox.Text);

            double zeroAngleMOA = Calculator.CalcVanillaZeroAngle(zeroRange, muzzleVelocity, airFriction, deltaT);
            SimulationRecord arcData = Calculator.BuildRangeTable(zeroAngleMOA, muzzleVelocity, airFriction, deltaT, maxRange).ReInterpolate(stepSize);
            ArcPlot.Plot.AddScatter(arcData.XPositions.ToArray(), arcData.YPositions.ToArray());
            string results = $"Zero angle: {zeroAngleMOA} MOA{Environment.NewLine}Terminal velocity: {arcData.AbsVelocities.Last()} m/s{Environment.NewLine}Time of Flight: {arcData.Timestamps.Last()} s";
            ResultsTextBox.Text = results;
            LastRecord = arcData;
        }

        private void ClearPlotButton_Click(object sender, RoutedEventArgs e)
        {
            ArcPlot.Plot.Clear();
        }

        private void WriteCSVButton_Click(object sender, RoutedEventArgs e)
        {
            if (LastRecord != null)
            {
                LastRecord.ExportToCSV("out.csv", 25.0);
            }
        }
    }
}
