package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.radian
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput
import org.ghrobotics.lib.utils.Source

class VisionAssistedTrajectoryTrackerCommand(
    val trajectorySource: Source<Trajectory<Time, TimedEntry<Pose2dWithCurvature>>>,
    val outerRadius: Length,
    val innerRadius: Length
) : FalconCommand(DriveSubsystem) {

    private var trajectoryFinished = false
    private var endpoint = Pose2d()
    private val trajectoryTracker = DriveSubsystem.trajectoryTracker

    var target = endpoint

    init {
        finishCondition += ::trajectoryFinished
    }

    override suspend fun initialize() {
        val trajectory = trajectorySource()

        endpoint = trajectory.lastState.state.pose
        target = endpoint
        trajectoryTracker.reset(trajectory)

        trajectoryFinished = false
        LiveDashboard.isFollowingPath = true
    }

    @Suppress("NestedBlockDepth")
    override suspend fun execute() {
        trajectoryFinished = trajectoryTracker.isFinished

        val referencePoint = trajectoryTracker.referencePoint

        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian

            // We will now use Vision to correct the robot pose.
            if (referencePose.distance(endpoint) < outerRadius.value) {
                if  (referencePose.distance(endpoint) > innerRadius.value) {
                    val bestTargetFromTargetTracker = TargetTracker.bestTarget
                    if (bestTargetFromTargetTracker != null) {
                        target = bestTargetFromTargetTracker.averagePose
                    } else {
                        println("LOST VISION")
                    }
                }

                val transform = target inFrameOfReferenceOf DriveSubsystem.robotPosition

                val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

                println(transform.translation.y.value)

                val linear = referencePoint.state.velocity
                val angular = kCorrectionKp * transform.translation.y.value * linear.value

                val output = TrajectoryTrackerOutput(
                    linearVelocity = linear,
                    angularVelocity = angular.radian.velocity,
                    linearAcceleration = referencePoint.state.acceleration,
                    angularAcceleration = (angular / linear.value * referencePoint.state.acceleration.value).radian.acceleration
                )

                DriveSubsystem.setOutput(output)
                trajectoryTracker.nextState(DriveSubsystem.robotPosition)
                return
            }

        }

        DriveSubsystem.setOutput(trajectoryTracker.nextState(DriveSubsystem.robotPosition))
    }

    override suspend fun dispose() {
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
    }

    companion object {
        const val kCorrectionKp = 2.0
        var isActive = false
            private set
    }

}