package org.ghrobotics.frc2019.robot.subsytems.drive

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet

/**
 * Class to generate splines on the fly for aligning with vision targets in the smoothest possible manner.
 */
object DriveOTFSplineGenerator {

    // Teleop on-the-fly constraints
    private val kOTFMaxVelocity = 10.feet.velocity
    private val kOTFMaxAcceleration = 4.0.feet.acceleration
    private val kOTFMaxCentripetalAcceleration = 4.5.feet.acceleration

     private val kOTFConstraints = listOf(CentripetalAccelerationConstraint(kOTFMaxCentripetalAcceleration))

    // Estimates the velocity lost when generating the spline and no human inputs are given
    private val kSplineGenerationDelayDeceleration = 0.2.feet.velocity

    // Creates a trajectory generator with lower tolerances than the default trajectory generator used for autonomous.
    // This allows us to generate trajectories on-the-fly quicker.
    private val lowToleranceTrajectoryGenerator = DefaultTrajectoryGenerator

    /**
     * Creates a trajectory with the current position as the starting point and [endpoint] as the end point.
     */
    fun create(endpoint: Pose2d): TimedTrajectory<Pose2dWithCurvature> {
        val waypoints = listOf(DriveSubsystem.localization(), endpoint)
        val startVelocity = (DriveSubsystem.leftMotor.velocity + DriveSubsystem.rightMotor.velocity) / 2.0 -
            kSplineGenerationDelayDeceleration

        return lowToleranceTrajectoryGenerator.generateTrajectory(
            waypoints,
            kOTFConstraints,
            startVelocity,
            0.0.feet.velocity,
            kOTFMaxVelocity,
            kOTFMaxAcceleration,
            false
        )
    }
}