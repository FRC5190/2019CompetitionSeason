package org.ghrobotics.frc2019.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.robot.vision.TargetTracker
import org.ghrobotics.frc2019.robot.vision.TrackedTarget
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricQuinticHermiteSpline
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricSplineGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.DistanceIterator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.DistanceTrajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import kotlin.math.sin
import kotlin.math.sqrt

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var iterator: DistanceIterator<Pose2dWithCurvature>
    lateinit var prevDistance: Length
    lateinit var prevVelocity: DifferentialDrive.ChassisState
    private var trackedTarget: TrackedTarget? = null
    private var lastEndPose: Pose2d? = null

    init {
        executeFrequency = 50
        finishCondition += { trackedTarget?.isAlive != true }
    }

    override suspend fun initialize() {
        val trackedTarget = TargetTracker.bestTarget
        this.trackedTarget = trackedTarget
        if (trackedTarget == null) return
        regenPath()
    }

    private fun updatePath() {
        val lastEndPose = this.lastEndPose!!
        val newEndPose = this.trackedTarget!!.averagePose
        if (lastEndPose.translation.distance(newEndPose.translation) > 0.1) {
            this.lastEndPose = newEndPose
            regenPath()
        }
    }

    private fun regenPath() {
        val endPose = this.lastEndPose ?: return
        iterator = DistanceTrajectory(
            ParametricSplineGenerator.parameterizeSplines(
                listOf(
                    ParametricQuinticHermiteSpline(
                        DriveSubsystem.localization(),
                        endPose + org.ghrobotics.frc2019.Constants.kForwardIntakeToCenter
                    )
                ),
                kMaxDx.value, kMaxDy.value,
                kMaxDTheta
            )
        ).iterator()
        prevDistance = DriveSubsystem.distanceTraveled
        prevVelocity = DifferentialDrive.ChassisState(0.0, 0.0)
    }

    override suspend fun execute() {
        updatePath()
        if (lastEndPose == null) return
        val distance = DriveSubsystem.distanceTraveled
        val dx = distance - prevDistance
        prevDistance = distance

        val reference = iterator.advance(dx)

        // Find Reference
        val vd =
            DriveSubsystem.voltageToSIVelocity(-ManualDriveCommand.speedSource() * 12.0)
        val wd = vd * reference.state.curvature.curvature.value

        // Compute Ramsete Error and Gains
        val error = reference.state.pose inFrameOfReferenceOf DriveSubsystem.localization()
        val k1 = 2 * org.ghrobotics.frc2019.Constants.kDriveZeta * sqrt(wd * wd + org.ghrobotics.frc2019.Constants.kDriveZeta * vd * vd)

        // Solve for Adjusted Angular Velocity
        val adjustedAngularVelocity =
            wd +
                org.ghrobotics.frc2019.Constants.kDriveBeta * vd * sinc(
                error.rotation.radian
            ) * error.translation.y.value +
                k1 * error.rotation.radian

        val velocity = DifferentialDrive.ChassisState(vd, adjustedAngularVelocity)
        val acceleration = DifferentialDrive.ChassisState(
            (velocity.linear - prevVelocity.linear) * executeFrequency,
            (velocity.angular - prevVelocity.angular) * executeFrequency
        )
        prevVelocity = velocity

        // Set outputs
        val dynamics = DriveSubsystem.differentialDrive.solveInverseDynamics(velocity, acceleration)
        DriveSubsystem.setOutput(dynamics.wheelVelocity, dynamics.voltage)
    }

    companion object {
        val kMaxDx = 2.inch
        val kMaxDy = 0.25.inch
        val kMaxDTheta = 5.degree

        private fun sinc(theta: Double) =
            if (theta epsilonEquals 0.0) {
                1.0 - 1.0 / 6.0 * theta * theta
            } else sin(theta) / theta
    }
}