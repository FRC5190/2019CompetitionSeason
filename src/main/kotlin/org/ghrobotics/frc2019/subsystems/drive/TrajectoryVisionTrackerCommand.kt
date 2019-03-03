package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput
import org.ghrobotics.lib.utils.Source

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param driveSubsystem Instance of the drive subsystem to use
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class TrajectoryVisionTrackerCommand(
    val trajectorySource: Source<Trajectory<Time, TimedEntry<Pose2dWithCurvature>>>,
    val radiusFromEnd: Length
) : FalconCommand(DriveSubsystem) {

    private var trajectoryFinished = false
    private var hasGeneratedVisionPath = false

    @Suppress("LateinitUsage")
    private lateinit var trajectory: Trajectory<Time, TimedEntry<Pose2dWithCurvature>>

    init {
        finishCondition += { trajectoryFinished }
    }

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override suspend fun initialize() {
        trajectory = trajectorySource()
        DriveSubsystem.trajectoryTracker.reset(trajectory)
        trajectoryFinished = false
        hasGeneratedVisionPath = false
        LiveDashboard.isFollowingPath = true
    }

    private var currentTarget: TargetTracker.TrackedTarget? = null
    private var lastKnownPose: Pose2d? = null

    override suspend fun execute() {
        val nextState = DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition)

        val withinVisionRadius =
            DriveSubsystem.localization().translation.distance(trajectory.lastState.state.pose.translation) < radiusFromEnd.value

        if (withinVisionRadius) {
            val newTarget = (if (!trajectory.reversed) {
                TargetTracker.bestTargetFront
            } else TargetTracker.bestTargetBack)

            if (newTarget != null) currentTarget = newTarget

            val newPose = currentTarget?.averagedPose2d
            if (currentTarget?.isAlive == true && newPose != null) lastKnownPose = newPose
        }

        val lastKnownVisionPose = this.lastKnownPose

        if (lastKnownVisionPose != null) {
            println("VISION")

            val transform = lastKnownVisionPose inFrameOfReferenceOf DriveSubsystem.localization()
            val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

//            val turn =
//                kCorrectionKp * (transform.translation.y.value / transform.translation.x.value.absoluteValue) * (if (targetSide == TargetSide.FRONT) 1.0 else -1.0)
            val turn =
                kCorrectionKp * (angle + if (!trajectory.reversed) Rotation2d.kZero else Math.PI.radian).radian
            DriveSubsystem.setOutput(
                TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    0.meter.acceleration,
                    turn.radian.velocity,
                    0.radian.acceleration
                )
            )
        } else {
            DriveSubsystem.setOutput(nextState)
        }

        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }

        trajectoryFinished = DriveSubsystem.trajectoryTracker.isFinished
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override suspend fun dispose() {
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
        visionActive = false
    }

    companion object {
        const val kCorrectionKp = 3.5
        var visionActive = false
    }
}
