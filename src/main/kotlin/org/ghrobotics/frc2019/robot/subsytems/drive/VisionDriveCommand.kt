package org.ghrobotics.frc2019.robot.subsytems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.Controls
import org.ghrobotics.frc2019.robot.vision.VisionProcessing
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
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.sin
import kotlin.math.sqrt

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var iterator: DistanceIterator<Pose2dWithCurvature>
    lateinit var prevDistance: Length
    lateinit var prevVelocity: DifferentialDrive.ChassisState
    private var endPose: Pose2d? = null

    init {
        executeFrequency = 50
        finishCondition += { endPose == null }
    }

    override suspend fun initialize() {
        val endPose = VisionProcessing.currentBestTarget
        this.endPose = endPose
        if (endPose == null) return
        regenPath()
    }

    private fun updatePath() {
        val endPose = this.endPose ?: return
        val newEndPose = VisionProcessing.currentlyTrackedObjects.minBy {
            it.translation.distance(endPose.translation)
        }
        if (newEndPose == null) {
            this.endPose = null
            return
        }
        if (endPose.translation.distance(newEndPose.translation) > 0.25) {
            this.endPose = newEndPose
            regenPath()
        }
    }

    private fun regenPath() {
        val endPose = this.endPose ?: return
        iterator = DistanceTrajectory(
            ParametricSplineGenerator.parameterizeSplines(
                listOf(
                    ParametricQuinticHermiteSpline(
                        DriveSubsystem.localization(),
                        endPose
                    )
                ),
                kMaxDx.value, kMaxDy.value, kMaxDTheta
            )
        ).iterator()
        prevDistance = DriveSubsystem.distanceTraveled
        prevVelocity = DifferentialDrive.ChassisState(0.0, 0.0)
    }

    override suspend fun execute() {
        updatePath()
        if (endPose == null) return
        val distance = DriveSubsystem.distanceTraveled
        val dx = distance - prevDistance
        prevDistance = distance

        val reference = iterator.advance(dx)

        // Find Reference
        val voltage = -Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(0.02)() * 12
        val predictedAcceleration = DriveSubsystem.differentialDrive.solveForwardDynamics(
            DifferentialDrive.WheelState(
                DriveSubsystem.leftMotor.velocity.value,
                DriveSubsystem.rightMotor.velocity.value
            ),
            DifferentialDrive.WheelState(voltage, voltage)
        ).chassisAcceleration

        val vd = DriveSubsystem.velocity.value + predictedAcceleration.linear * executeFrequency

        val wd = vd * reference.state.curvature.curvature.value

        // Compute Ramsete Error and Gains
        val error = reference.state.pose inFrameOfReferenceOf DriveSubsystem.localization()
        val k1 = 2 * Constants.kDriveZeta * sqrt(wd * wd + Constants.kDriveZeta * vd * vd)

        // Solve for Adjusted Angular Velocity
        val adjustedAngularVelocity =
            wd +
                Constants.kDriveBeta * vd * sinc(error.rotation.radian) * error.translation.y.value +
                k1 * error.rotation.radian

        val velocity = DifferentialDrive.ChassisState(vd, adjustedAngularVelocity)
        val acceleration = DifferentialDrive.ChassisState(
            (velocity.linear - prevVelocity.linear) * executeFrequency,
            (velocity.angular - prevVelocity.angular) * executeFrequency
        )
        prevVelocity = velocity

        // Set outputs
        val dynamics = DriveSubsystem.differentialDrive.solveInverseDynamics(velocity, acceleration)
        DriveSubsystem.leftMotor.percentOutput = dynamics.voltage.left / 12.0
        DriveSubsystem.rightMotor.percentOutput = dynamics.voltage.right / 12.0
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