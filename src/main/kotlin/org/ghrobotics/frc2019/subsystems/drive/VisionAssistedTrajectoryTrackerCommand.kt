package org.ghrobotics.frc2019.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.utils.Source
import kotlin.math.sin

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
                if (referencePose.distance(endpoint) > innerRadius.value) {
                    val bestTargetFromTargetTracker =
                        TargetTracker.trackedTargets.minBy { DriveSubsystem.robotPosition.distance(it.averagePose) }
                    if (bestTargetFromTargetTracker != null) {
                        target = bestTargetFromTargetTracker.averagePose
                        println("USING VISION")
                        isActive = true
                    } else {
                        isActive = false
                    }
                }

                val transform = target inFrameOfReferenceOf DriveSubsystem.robotPosition


//                val vd = referencePoint.state.velocity.value
//                val wd = vd * referencePoint.state.state.curvature.curvature.value
//
//                val k1 = 2 * kVisionZeta * sqrt(wd * wd + kVisionBeta * vd * vd)
//                val angular = kVisionBeta * vd * sinc(transform.rotation.radian) * transform.translation.y.value +
//                    k1 * transform.rotation.value

                val linear = referencePoint.state.velocity.value
                val angular = kCorrectionKp * transform.translation.y.value * linear
//
//
                DriveSubsystem.setOutputFromDynamics(
                    chassisVelocity = DifferentialDrive.ChassisState(linear, angular),
                    chassisAcceleration = DifferentialDrive.ChassisState(0.0, 0.0)
                )
                trajectoryTracker.nextState(DriveSubsystem.robotPosition)
                return
            }

        }

        DriveSubsystem.setOutput(trajectoryTracker.nextState(DriveSubsystem.robotPosition))
    }

    override suspend fun dispose() {
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
        isActive = false
    }

    companion object {
        const val kVisionBeta = 1.0
        const val kVisionZeta = 0.5

        private fun sinc(theta: Double) =
            if (theta epsilonEquals 0.0) {
                1.0 - 1.0 / 6.0 * theta * theta
            } else sin(theta) / theta

        const val kCorrectionKp = 1.0
        var isActive = false
            private set
    }

}