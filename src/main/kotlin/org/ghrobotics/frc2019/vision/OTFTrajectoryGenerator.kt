package org.ghrobotics.frc2019.vision

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter

object OTFTrajectoryGenerator {

    private val kMaxVelocity = 6.feet.velocity
    private val kMaxAcceleration = 4.feet.acceleration

    fun generateOTFTrajectory(
        target: TargetTracker.TrackedTarget,
        reversed: Boolean,
        endpointAngle: Rotation2d
    ): TimedTrajectory<Pose2dWithCurvature> {
        val endpoint = Pose2d((target.averagedPose2d + Constants.kForwardIntakeToCenter).translation, endpointAngle)

        val waypoints = listOf(
            DriveSubsystem.localization(),
            endpoint + Pose2d((-6).inch, 0.inch),
            endpoint
        )
        return DefaultTrajectoryGenerator.generateTrajectory(
            waypoints,
            listOf(),
            DriveSubsystem.velocity.meter.velocity,
            0.feet.velocity,
            kMaxVelocity,
            kMaxAcceleration,
            reversed
        )
    }
}