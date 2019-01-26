/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.OpenLoopArmCommand
import org.ghrobotics.frc2019.subsystems.climb.ClimbSubsystem
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakePneumaticCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeWheelCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.pow
import kotlin.math.withSign

object Controls {

    var isClimbing = false
        private set

    val driverXbox = xboxController(0) {
        registerEmergencyMode()

        state({ !isClimbing }) {
            // Vision align
            button(kBumperRight).change(VisionDriveCommand())

            // Intake
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
        registerEmergencyMode()

        // Enter climb mode
        button(kB).changeOn { isClimbing = !isClimbing }

        state({ !isClimbing }) {
            // Elevator
            axisButton(1, 0.05) {
                change(OpenLoopElevatorCommand(source.map {  it.pow(2).withSign(-it) * .5 }))
            }
            // Arm
            axisButton(2, 0.05) {
                change(OpenLoopArmCommand(source.map { it.pow(2).withSign(-it) * .5 }))
            }

            // Superstructure
            pov(0).changeOn(Superstructure.kFrontHighRocketHatch)
            pov(90).changeOn(Superstructure.kFrontMiddleRocketHatch)
            pov(180).changeOn(Superstructure.kFrontLoadingStation)
            pov(270).changeOn(ClosedLoopElevatorCommand(0.inch))
        }
        state({ isClimbing }) {
            button(kA).change(sequential {
                // Auto climbing logic here
            })

            // Nihar xd
            button(kY).changeOn { ClimbSubsystem.ramps = true }
        }
    }

    private fun FalconXboxBuilder.registerEmergencyMode() {
        button(kBack).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.activateEmergency() }
            Robot.emergencyActive = true
        }
        button(kStart).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.recoverFromEmergency() }
            Robot.emergencyActive = false
        }
    }

    fun update() {
        driverXbox.update()
        operatorXbox.update()
    }
}