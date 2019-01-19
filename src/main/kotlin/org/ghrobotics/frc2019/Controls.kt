/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.arm.OpenLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakePneumaticCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeWheelCommand
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.pow
import kotlin.math.withSign

object Controls {

    var isClimbing = false
        private set

    val driverXbox = xboxController(0) {
        state({ !isClimbing }) {
            // Align with Vision Target
            button(kB).change(VisionDriveCommand())

            // Emergency Mode
            button(kBack).changeOn {
                Robot.emergencyReadySystems.forEach { system -> system.activateEmergency() }
                Robot.emergencyActive = true
            }
            button(kStart).changeOn {
                Robot.emergencyReadySystems.forEach { system -> system.recoverFromEmergency() }
                Robot.emergencyActive = false
            }
            button(kBumperRight).change(VisionDriveCommand())

            triggerAxisButton(GenericHID.Hand.kRight) {
                change(IntakeWheelCommand(IntakeSubsystem.Direction.IN))
            }
            triggerAxisButton(GenericHID.Hand.kLeft) {
                change(IntakeWheelCommand(IntakeSubsystem.Direction.OUT))
            }
            button(kX).change(IntakePneumaticCommand { !IntakeSubsystem.solenoid.get() })
        }
    }

    val operatorXbox = xboxController(1) {
        axisButton(1, 0.05) {
            change(OpenLoopElevatorCommand(source.map { -it * 0.5 }))
        }
        axisButton(2, 0.05) {
            change(OpenLoopArmCommand(source.map { it.pow(2).withSign(-it) * .5 }))
        }
    }

    fun update() {
        driverXbox.update()
        operatorXbox.update()
    }
}