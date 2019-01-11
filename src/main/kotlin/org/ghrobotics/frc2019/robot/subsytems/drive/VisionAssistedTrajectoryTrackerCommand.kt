package org.ghrobotics.frc2019.robot.subsytems.drive

import org.ghrobotics.frc2019.robot.auto.VisionAssistedTrajectory
import org.ghrobotics.frc2019.robot.vision.DynamicObject
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.utils.Source

class VisionAssistedTrajectoryTrackerCommand(
    val trajectorySource: Source<TimedTrajectory<Pose2dWithCurvature>>,
    val dynamicObject: Source<DynamicObject>,
    val expectedLocation: Source<Translation2d>,
    val error: Source<Length>,
    dt: Time = DriveSubsystem.kPathFollowingDt
) : FalconCommand(DriveSubsystem) {

    private var trajectoryFinished = false
    private val trajectoryTracker = DriveSubsystem.trajectoryTracker

    init {
        finishCondition += { trajectoryFinished }
        executeFrequency = (1 / dt.second).toInt()
    }

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override suspend fun initialize() {
        trajectoryTracker.reset(
            VisionAssistedTrajectory(
                trajectorySource(),
                dynamicObject(),
                expectedLocation(),
                error()
            )
        )
        trajectoryFinished = false
        LiveDashboard.isFollowingPath = true
    }

    override suspend fun execute() {
        DriveSubsystem.setOutput(trajectoryTracker.nextState(DriveSubsystem.robotPosition))

        val referencePoint = trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }

        trajectoryFinished = trajectoryTracker.isFinished
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override suspend fun dispose() {
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
    }

}