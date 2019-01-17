/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    val mainXbox = xboxController(0) {

        // Align with Vision Target
        triggerAxisButton(GenericHID.Hand.kLeft).change(VisionDriveCommand())

        // Emergency Mode
        button(kBack).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.activateEmergency() }
        }
        button(kStart).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.recoverFromEmergency() }
        }

        // Elevator Controls
        val elevatorUpCommand = OpenLoopElevatorCommand(0.4)
        val elevatorDownCommand = OpenLoopElevatorCommand(-0.4)

        triggerAxisButton(GenericHID.Hand.kRight, 0.2).change(elevatorUpCommand)
        button(kBumperRight).change(elevatorDownCommand)

        button(kA).changeOn { DriveSubsystem.lowGear = true }
        button(kA).changeOff { DriveSubsystem.lowGear = false }
    }

    fun update() {
        mainXbox.update()
    }
}