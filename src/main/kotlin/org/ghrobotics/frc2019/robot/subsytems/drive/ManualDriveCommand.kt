/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.subsytems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX
import kotlin.math.absoluteValue
import kotlin.math.max

class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    private var quickStopAccumulator = 0.0
    private var prevVelocity = DifferentialDrive.WheelState(0.0, 0.0)

    init {
        executeFrequency = 50
    }

    override suspend fun initialize() {
        prevVelocity = DifferentialDrive.WheelState(0.0, 0.0)
    }

    override suspend fun execute() {
        this.closedLoopCurvatureDrive(
            -speedSource(),
            rotationSource(),
            quickTurnSource()
        )
    }

    private fun closedLoopCurvatureDrive(
        linearPercent: Double,
        curvaturePercent: Double,
        isQuickTurn: Boolean
    ) {
        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (linearPercent.absoluteValue < TankDriveSubsystem.kQuickStopThreshold) {
                quickStopAccumulator = (1 - TankDriveSubsystem.kQuickStopAlpha) * quickStopAccumulator +
                    TankDriveSubsystem.kQuickStopAlpha * curvaturePercent.coerceIn(-1.0, 1.0) * 2.0
            }
            overPower = true
            angularPower = curvaturePercent
        } else {
            overPower = false
            angularPower = linearPercent.absoluteValue * curvaturePercent - quickStopAccumulator

            when {
                quickStopAccumulator > 1 -> quickStopAccumulator -= 1.0
                quickStopAccumulator < -1 -> quickStopAccumulator += 1.0
                else -> quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = linearPercent + angularPower
        var rightMotorOutput = linearPercent - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            when {
                leftMotorOutput > 1.0 -> {
                    rightMotorOutput -= leftMotorOutput - 1.0
                    leftMotorOutput = 1.0
                }
                rightMotorOutput > 1.0 -> {
                    leftMotorOutput -= rightMotorOutput - 1.0
                    rightMotorOutput = 1.0
                }
                leftMotorOutput < -1.0 -> {
                    rightMotorOutput -= leftMotorOutput + 1.0
                    leftMotorOutput = -1.0
                }
                rightMotorOutput < -1.0 -> {
                    leftMotorOutput -= rightMotorOutput + 1.0
                    rightMotorOutput = -1.0
                }
            }
        }

        // Normalize the wheel speeds
        val maxMagnitude = max(leftMotorOutput.absoluteValue, rightMotorOutput.absoluteValue)
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude
            rightMotorOutput /= maxMagnitude
        }

        val velocity = DifferentialDrive.WheelState(
            DriveSubsystem.voltageToSIVelocity(leftMotorOutput * 12.0),
            DriveSubsystem.voltageToSIVelocity(rightMotorOutput * 12.0)
        )
        val acceleration = DifferentialDrive.WheelState(
            (velocity.left - prevVelocity.left) * executeFrequency,
            (velocity.right - prevVelocity.right) * executeFrequency
        )
        prevVelocity = velocity

        val dynamics = DriveSubsystem.differentialDrive.solveInverseDynamics(velocity, acceleration)
        DriveSubsystem.setOutput(dynamics.wheelVelocity, dynamics.voltage)
    }

    companion object {
        private const val deadband = 0.02

        val speedSource = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(deadband)

        private val rotationSource = Controls.mainXbox.getX(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val quickTurnSource = Controls.mainXbox.getRawButton(kX)
    }
}
