package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.vision.OTFTrajectoryGenerator
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
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

    override suspend fun execute() {
        val visionTarget = if (trajectory.reversed) TargetTracker.bestTargetBack else TargetTracker.bestTargetFront

        if (DriveSubsystem.localization().translation.distance(trajectory.lastState.state.pose.translation) < radiusFromEnd.value
            && visionTarget != null
            && !hasGeneratedVisionPath
        ) {
            hasGeneratedVisionPath = true
            trajectory = OTFTrajectoryGenerator.generateOTFTrajectory(
                visionTarget,
                trajectory.reversed,
                trajectory.lastState.state.pose.rotation
            )
            DriveSubsystem.trajectoryTracker.reset(trajectory)
        }

        DriveSubsystem.setOutput(DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition))

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
    }
}
