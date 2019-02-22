/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.withSign

class ManualDriveCommand : FalconCommand(DriveSubsystem) {

//    private var armStowedRange = (90.degree - Constants.kArmFlipTolerance)..(90.degree + Constants.kArmFlipTolerance)

    override suspend fun execute() {
//        DriveSubsystem.tankDrive(
//            -leftSource(),
//            -rightSource()
//        )
        // TODO add conditions that make placing the hatch easier
        val curvature = rotationSource()
        val linear = -speedSource()
        if (curvature != 0.0 || linear != 0.0) {
            curvatureDrive(
                linear,
                curvature,
                quickTurnSource()
            )
        } else if (DriveSubsystem.leftMotor.velocity.absoluteValue < kLockVelocityTolerance
            && DriveSubsystem.rightMotor.velocity.absoluteValue < kLockVelocityTolerance
        ) {
            DriveSubsystem.leftMotor.set(ControlMode.Velocity, 0.0)
            DriveSubsystem.rightMotor.set(ControlMode.Velocity, 0.0)
        }
    }

    /**
     * Curvature or cheezy drive control
     */
    @Suppress("ComplexMethod")
    private fun curvatureDrive(
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

        tankDrive(leftMotorOutput, rightMotorOutput)
    }

    /**
     * Tank drive control
     */
    private fun tankDrive(
        leftPercent: Double,
        rightPercent: Double
    ) {
        DriveSubsystem.leftMotor.set(
            ControlMode.PercentOutput,
            leftPercent,
            DemandType.ArbitraryFeedForward,
            Constants.kDriveLeftKs.withSign(leftPercent) / 12.0
        )
        DriveSubsystem.rightMotor.set(
            ControlMode.PercentOutput,
            rightPercent,
            DemandType.ArbitraryFeedForward,
            Constants.kDriveRightKs.withSign(rightPercent) / 12.0
        )
    }

    companion object {
        private var kLockVelocityTolerance = 0.5.feet.velocity
        private var quickStopAccumulator = 0.0
        private const val kQuickStopThreshold = TankDriveSubsystem.kQuickStopThreshold
        private const val kQuickStopAlpha = TankDriveSubsystem.kQuickStopAlpha
        private const val kDeadband = 0.02
        val speedSource = Controls.driverXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        val rightSource = Controls.driverXbox.getY(GenericHID.Hand.kRight).withDeadband(kDeadband)
        val leftSource = Controls.driverXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val rotationSource = Controls.driverXbox.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val quickTurnSource = Controls.driverXbox.getRawButton(kX)
    }
}
