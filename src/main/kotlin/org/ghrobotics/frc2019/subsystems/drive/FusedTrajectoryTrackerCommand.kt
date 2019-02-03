package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

class FusedTrajectoryTrackerCommand(
    val trajectorySource: Source<Trajectory<Time, TimedEntry<Pose2dWithCurvature>>>,
    val visionLocalizationUpdateStart: Time,
    val visionStaticObjectLocation: Source<Pose2d>
) : FalconCommand(DriveSubsystem) {

    private var trajectoryFinished = false
    private var duration = 0.second

    init {
        finishCondition += { trajectoryFinished }
        executeFrequency = (1 / DriveSubsystem.kPathFollowingDt.second).toInt()
    }

    override suspend fun initialize() {
        val trajectory = trajectorySource()
        DriveSubsystem.trajectoryTracker.reset(trajectory)
        duration = trajectory.duration
        trajectoryFinished = false
        LiveDashboard.isFollowingPath = true
    }

    override suspend fun execute() {
        DriveSubsystem.setOutput(DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition))
        val trackedTarget = TargetTracker.bestTarget


        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            if (duration - referencePoint.state.t < visionLocalizationUpdateStart && trackedTarget != null) {
                DriveSubsystem.localization.addVisionSample(trackedTarget, visionStaticObjectLocation())
            }

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }

        trajectoryFinished = DriveSubsystem.trajectoryTracker.isFinished
    }
}