package org.ghrobotics.frc2019.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.frc2019.vision.TrackedTarget
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricQuinticHermiteSpline
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricSplineGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.DistanceTrajectory
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.sin
import kotlin.math.sqrt

class VisionDriveCommand2 : FalconCommand(DriveSubsystem) {

    lateinit var target: TrackedTarget
    private var foundTarget = false

    var previousDistance = 0.meter
    var prevVelocity = DifferentialDrive.ChassisState(0.0, 0.0)

    init {
        finishCondition += { !foundTarget }
        executeFrequency = 10
    }

    override suspend fun initialize() {
        val target = TargetTracker.bestTarget
        if (target == null) {
            foundTarget = false
        } else {
            this.target = target
            foundTarget = true
        }
    }

    override suspend fun execute() {
        val iterator = DistanceTrajectory(
            ParametricSplineGenerator.parameterizeSplines(
                listOf(
                    ParametricQuinticHermiteSpline(
                        DriveSubsystem.localization(),
                        target.averagePose
                    )
                ),
                kMaxDx.value, kMaxDy.value, kMaxDTheta
            )
        ).iterator()

        val distanceTraveled =
            ((DriveSubsystem.leftMotor.sensorPosition + DriveSubsystem.rightMotor.sensorPosition) / 2.0)

        val reference = iterator.advance(distanceTraveled - previousDistance)
        previousDistance = distanceTraveled

        val source = -ManualDriveCommand.speedSource() * 12

        val voltage = DifferentialDrive.WheelState(source, source)
        val currentVelocity = DifferentialDrive.WheelState(
            DriveSubsystem.leftMotor.sensorVelocity.value / Constants.kWheelRadius.value,
            DriveSubsystem.rightMotor.sensorVelocity.value / Constants.kWheelRadius.value
        )

        val predictedAcceleration =
            DriveSubsystem.differentialDrive.solveForwardDynamics(currentVelocity, voltage).chassisAcceleration

        val vd =
            ((DriveSubsystem.leftMotor.sensorVelocity + DriveSubsystem.rightMotor.sensorVelocity) / 2).value +
                predictedAcceleration.linear / executeFrequency

        val wd = vd * reference.state.curvature.curvature.value

        val error = reference.state.pose inFrameOfReferenceOf DriveSubsystem.localization()
        val k1 = 2 * Constants.kDriveZeta * sqrt(wd * wd + Constants.kDriveZeta * vd * vd)

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