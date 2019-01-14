package org.ghrobotics.frc2019.robot.subsytems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.Controls
import org.ghrobotics.frc2019.robot.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricQuinticHermiteSpline
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricSplineGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.DistanceIterator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.DistanceTrajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.sin
import kotlin.math.sqrt

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var iterator: DistanceIterator<Pose2dWithCurvature>
    lateinit var prevDistance: Length
    lateinit var prevVelocity: DifferentialDrive.ChassisState
    private var isFinished = false

    init {
        executeFrequency = 50
        finishCondition += ::isFinished
    }

    override suspend fun initialize() {
        val endPose = VisionProcessing.currentBestTarget
        if (endPose == null) {
            isFinished = true
            return
        } else {
            isFinished = false
        }
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
        if (isFinished) return
        val distance = DriveSubsystem.distanceTraveled
        val dx = distance - prevDistance
        prevDistance = distance

        val reference = iterator.advance(dx)

        // Find Reference
        // TODO Figure out best way to get vd
        val vd = -Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(0.02)() * 17.feet.velocity.value
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