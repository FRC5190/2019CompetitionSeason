package org.ghrobotics.frc2019.vision

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet

object OTFTrajectoryGenerator {

    private val kMaxVelocity = 6.feet.velocity
    private val kMaxAcceleration = 6.feet.acceleration

    fun generateForwardOTFTrajectory() = generateOTFTrajectory(TargetTracker.bestTargetFront, false)
    fun generateBackwardOTFTrajectory() = generateOTFTrajectory(TargetTracker.bestTargetBack, true)

    private fun generateOTFTrajectory(
        target: TrackedTarget?,
        reversed: Boolean
    ): TimedTrajectory<Pose2dWithCurvature>? {
        return if (target != null) {
            val waypoints = listOf(
                DriveSubsystem.localization(),
                target.averagePose + Constants.kForwardIntakeToCenter
            )
            DefaultTrajectoryGenerator.generateTrajectory(
                waypoints, listOf(), DriveSubsystem.velocity, 0.feet.velocity, kMaxVelocity, kMaxAcceleration, reversed
            )
        } else
            null
    }
}