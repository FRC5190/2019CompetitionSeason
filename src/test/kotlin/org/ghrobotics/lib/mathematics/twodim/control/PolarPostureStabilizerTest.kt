package org.ghrobotics.lib.mathematics.twodim.control

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.simulation.SimDifferentialDrive
import org.ghrobotics.lib.simulation.SimFalconMotor
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.awt.Color
import java.awt.Font

class PolarPostureStabilizerTest {
    @Test
    fun testPostureStabilizer() {
        val polarPostureStabilizer = PolarPostureStabilizer(1.0, 3.0, 2.0)

        val drive = SimDifferentialDrive(
            Constants.kDriveModel,
            SimFalconMotor(),
            SimFalconMotor(),
            FeedForwardTracker(),
            1.05
        )

        var currentTime = 0.second
        val deltaTime = 20.millisecond

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()


        val target = Pose2d(20.feet, 5.feet, 0.degree)

        polarPostureStabilizer.reset(target)

        drive.robotPosition = Pose2d(10.0.feet, 10.feet, 30.degree)

        while (currentTime < 12.second) {
            currentTime += deltaTime
            drive.setOutput(polarPostureStabilizer.nextState(drive.robotPosition, currentTime))
            drive.update(deltaTime)

            xList += drive.robotPosition.translation.x / SILengthConstants.kFeetToMeter
            yList += drive.robotPosition.translation.y / SILengthConstants.kFeetToMeter
        }

        val chart = XYChartBuilder().width(1800).height(1520).title("Polar Posture Stabilizer")
            .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = 1.0
        chart.styler.xAxisMax = 26.0
        chart.styler.yAxisMin = 1.0
        chart.styler.yAxisMax = 26.0

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

//        SwingWrapper(chart).displayChart()
//        Thread.sleep(1000000)
    }
}