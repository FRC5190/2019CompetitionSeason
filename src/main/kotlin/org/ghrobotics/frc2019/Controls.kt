/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakePneumaticCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeWheelCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.pow

object Controls {

    var isClimbing = false
        private set

    val mainXbox = xboxController(0) {
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

            // Elevator Controls
            val elevatorUpCommand = OpenLoopElevatorCommand(0.4)
            val elevatorDownCommand = OpenLoopElevatorCommand(-0.4)

            triggerAxisButton(GenericHID.Hand.kRight, 0.2).change(elevatorUpCommand)
            button(kBumperRight).change(elevatorDownCommand)

            button(kA).changeOn { DriveSubsystem.lowGear = true }
            button(kA).changeOff { DriveSubsystem.lowGear = false }

            // Intake Controls
            triggerAxisButton(GenericHID.Hand.kLeft, 0.1) {
                change(IntakeWheelCommand(IntakeSubsystem.Direction.OUT, source.map { it.pow(2) * 0.65 }))
            }
            button(kBumperLeft).change(sequential {
                +IntakeWheelCommand(IntakeSubsystem.Direction.IN, 1.0)
            })
            button(kY).changeOn(IntakePneumaticCommand { !IntakeSubsystem.solenoid.get() })
        }
    }

    fun update() {
        mainXbox.update()
    }
}