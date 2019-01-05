package org.ghrobotics.frc2019.sim.subsystems.drive

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.AngularAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.text.DecimalFormat

@Suppress("UnsafeCallOnNullableType")
// TODO Putting everything in Main doesn't feel right. We need to figure out something else.
object SimulateTrajectoryFollower {

    private val kMaxCentripetalAcceleration = 2.feet.acceleration
    private val kMaxAcceleration = 4.feet.acceleration
    private val kMaxAngularAcceleration = 2.radian.acceleration
    private val kMaxVelocity = 10.feet.velocity

    private val kSideStart = Pose2d(1.54.feet, 23.234167.feet, 180.degree)
    private val kNearScaleEmpty = Pose2d(23.7.feet, 20.2.feet, 160.degree)

    private val trajectory = DefaultTrajectoryGenerator.generateTrajectory(
        listOf(
            kSideStart,
            kSideStart + Pose2d((-13).feet, 0.feet, 0.degree),
            kSideStart + Pose2d((-19.5).feet, 5.feet, (-90).degree),
            kSideStart + Pose2d((-19.5).feet, 14.feet, (-90).degree),
            kNearScaleEmpty.mirror
        ),
        listOf(
            CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
            DifferentialDriveDynamicsConstraint(SimulatedDriveSubsystem.differentialDrive, 9.0.volt),
            AngularAccelerationConstraint(kMaxAngularAcceleration)
        ),
        0.0.feet.velocity,
        0.0.feet.velocity,
        kMaxVelocity,
        kMaxAcceleration,
        true
    )

    @JvmStatic
    fun main(args: Array<String>) {
        var currentTime = 0.second
        val deltaTime = 5.millisecond

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val refXList = arrayListOf<Double>()
        val refYList = arrayListOf<Double>()

        SimulatedDriveSubsystem.trajectoryTracker.reset(trajectory)

        SimulatedDriveSubsystem.robotLocation =
            SimulatedDriveSubsystem.trajectoryTracker.referencePoint!!.state.state.pose
                .transformBy(Pose2d(1.feet, 50.inch, 5.degree))

        while (!SimulatedDriveSubsystem.trajectoryTracker.isFinished) {
            currentTime += deltaTime
            val out =
                SimulatedDriveSubsystem.trajectoryTracker.nextState(SimulatedDriveSubsystem.robotLocation, currentTime)
            SimulatedDriveSubsystem.setOutput(out)

            xList += SimulatedDriveSubsystem.robotLocation.translation.x.feet
            yList += SimulatedDriveSubsystem.robotLocation.translation.y.feet

            val referenceTranslation =
                SimulatedDriveSubsystem.trajectoryTracker.referencePoint!!.state.state.pose.translation
            refXList += referenceTranslation.x.feet
            refYList += referenceTranslation.y.feet

        }

        val fm = DecimalFormat("#.###").format(trajectory.lastInterpolant.second)

        val chart = XYChartBuilder().title("$fm seconds.")
            .xAxisTitle("X").yAxisTitle("Y").build()


        chart.addSeries("Trajectory", refXList.toDoubleArray(), refYList.toDoubleArray())
        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

        val terror =
            trajectory.lastState.state.pose.translation - SimulatedDriveSubsystem.robotLocation.translation

        System.out.printf("%n[Test] X Error: %3.3f, Y Error: %3.3f%n", terror.x.feet, terror.y.feet)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }
}