package org.ghrobotics.frc2019.robot.auto

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.junit.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import kotlin.system.measureNanoTime

class TrajectoryGenerationTest {

    private val points = listOf(Pose2d(0.meter, 0.meter, 0.degree), Pose2d(2.meter, 3.meter, 120.degree))

    @Test
    fun compareTrajectoryGenerators() {
        testLowToleranceTrajectoryGenerator()
        testDefaultTrajectoryGenerator()
//        Thread.sleep(1000000)
    }

    private fun testLowToleranceTrajectoryGenerator() {
        val generator = TrajectoryGenerator(3.feet, 2.feet, 10.degree)
        lateinit var trajectory: TimedTrajectory<Pose2dWithCurvature>
        val time = measureNanoTime {
            trajectory = generator.generateTrajectory(
                points, listOf(), 3.feet.velocity, 0.feet.velocity, 10.feet.velocity, 5.feet.acceleration, false
            )
        }
        println("Low Tolerance Trajectory Generator Took: ${time.nanosecond.second} seconds.")
        visualize(trajectory, "Low Tolerance Trajectory Generator")
    }

    private fun testDefaultTrajectoryGenerator() {
        lateinit var trajectory: TimedTrajectory<Pose2dWithCurvature>
        val time = measureNanoTime {
            trajectory = DefaultTrajectoryGenerator.generateTrajectory(
                points, listOf(), 3.feet.velocity, 0.feet.velocity, 10.feet.velocity, 5.feet.acceleration, false
            )
        }
        println("Default Trajectory Generator Took: ${time.nanosecond.second} seconds.")
        visualize(trajectory, "Default Trajectory Generator")
    }

    fun visualize(trajectory: TimedTrajectory<Pose2dWithCurvature>, type: String) {
        val iterator = trajectory.iterator()

        val x = arrayListOf<Double>()
        val y = arrayListOf<Double>()

        while (!iterator.isDone) {
            val translation = iterator.advance(0.02.second).state.state.pose.translation
            x.add(translation.x.value)
            y.add(translation.y.value)
        }

//        SwingWrapper(QuickChart.getChart(type, "X", "Y", "Path", x.toDoubleArray(), y.toDoubleArray())).displayChart()
    }

}