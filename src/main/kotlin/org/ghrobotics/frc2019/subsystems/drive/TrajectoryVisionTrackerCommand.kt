package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
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
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class TrajectoryVisionTrackerCommand(
    val trajectorySource: Source<Trajectory<Time, TimedEntry<Pose2dWithCurvature>>>,
    val radiusFromEnd: Length,
    val useAbsoluteVision: Boolean = false
) : FalconCommand(DriveSubsystem) {

    private var trajectoryFinished = false
    private var hasGeneratedVisionPath = false

    private var prevError = 0.0

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

    private var lastKnownTargetPose: Pose2d? = null

    override suspend fun execute() {
        val robotPositionWithIntakeOffset = IntakeSubsystem.robotPositionWithIntakeOffset

        val nextState = DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition)

        val withinVisionRadius =
            robotPositionWithIntakeOffset.translation.distance(
                trajectory.lastState.state.pose.translation + Translation2d(
                    Length.kZero,
                    IntakeSubsystem.badIntakeOffset
                )
            ) < radiusFromEnd.value

        if (withinVisionRadius) {
            val newTarget = if (!useAbsoluteVision) {
                TargetTracker.getBestTarget(!trajectory.reversed)
            } else {
                TargetTracker.getAbsoluteTarget((trajectory.lastState.state.pose + Constants.kCenterToForwardIntake).translation)
            }

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose
        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null) {
            println("VISION")
            visionActive = true
            val transform = lastKnownTargetPose inFrameOfReferenceOf robotPositionWithIntakeOffset
            val angle = Rotation2d(transform.translation.x, transform.translation.y, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val error = (angle + if (!trajectory.reversed) Rotation2d.kZero else Math.PI.radian).radian
            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)


            DriveSubsystem.setOutput(
                TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    0.meter.acceleration,
                    turn.radian.velocity,
                    0.radian.acceleration
                )
            )

            prevError = error
            
        } else {
            DriveSubsystem.setOutput(nextState)
        }

        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x / SILengthConstants.kFeetToMeter
            LiveDashboard.pathY = referencePose.translation.y / SILengthConstants.kFeetToMeter
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
        const val kCorrectionKp = 5.5
        const val kCorrectionKd = 0.0
        var visionActive = false
    }
} 