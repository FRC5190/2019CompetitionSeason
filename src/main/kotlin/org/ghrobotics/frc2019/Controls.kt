/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.OpenLoopArmCommand
import org.ghrobotics.frc2019.subsystems.climb.autoL2Climb
import org.ghrobotics.frc2019.subsystems.climb.autoL3Climb
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
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
            button(kY).change(VisionDriveCommand(VisionDriveCommand.TargetSide.FRONT))
            button(kB).change(VisionDriveCommand(VisionDriveCommand.TargetSide.BACK))

            // Shifting
            button(kA).changeOn { DriveSubsystem.lowGear = true }
            button(kA).changeOff { DriveSubsystem.lowGear = false }

            // Intake
            triggerAxisButton(GenericHID.Hand.kLeft).change(IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE))
            button(kBumperLeft).change(IntakeHatchCommand(IntakeSubsystem.Direction.HOLD))

            triggerAxisButton(GenericHID.Hand.kRight).change(IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE))
            button(kBumperRight).change(IntakeCargoCommand(IntakeSubsystem.Direction.HOLD))
        }
    }

    val operatorXbox = xboxController(1) {
        registerEmergencyMode()

        // Enter climb mode
        button(kB).changeOn {
            isClimbing = !isClimbing
            DriveSubsystem.lowGear = true
            Superstructure.kStowedPosition.start()
        }

        state({ !isClimbing }) {
            // Elevator
            axisButton(1, 0.15) {
                change(OpenLoopElevatorCommand(source.map { it.pow(2).withSign(-it) * 0.5 }))
            }
            // Arm
            axisButton(5, 0.15) {
                change(OpenLoopArmCommand(source.map { it.pow(2).withSign(-it) * 0.5 }))
            }

            val backModifier = triggerAxisButton(GenericHID.Hand.kLeft)

            // Superstructure

            // HIGH NEAR_ROCKET
            pov(0).changeOn {
                if (IntakeSubsystem.isSeeingCargo()) {
                    Superstructure.kFrontHighRocketCargo.start()
                } else {
                    Superstructure.kFrontHighRocketHatch.start()
                }
            }

            // MIDDLE NEAR_ROCKET
            pov(90).changeOn {
                if (IntakeSubsystem.isSeeingCargo()) {
                    Superstructure.kFrontMiddleRocketCargo.start()
                } else {
                    Superstructure.kFrontMiddleRocketHatch.start()
                }
            }

            // LOW NEAR_ROCKET, CARGO SHIP, AND LOADING STATION
            pov(180).changeOn {
                if (backModifier.source() > 0.35) {
                    if (IntakeSubsystem.isSeeingCargo()) {
                        Superstructure.kBackLowRocketCargo.start()
                    } else {
                        Superstructure.kBackHatchFromLoadingStation.start()
                    }
                } else {
                    if (IntakeSubsystem.isSeeingCargo()) {
                        Superstructure.kFrontLowRocketCargo.start()
                    } else {
                        Superstructure.kFrontHatchFromLoadingStation.start()
                    }
                }
            }

            // CARGO INTAKE
            pov(270).changeOn {
                if (backModifier.source() > 0.35) {
                    Superstructure.kBackCargoIntake.start()
                } else {
                    Superstructure.kFrontCargoIntake.start()
                }
            }

            triggerAxisButton(GenericHID.Hand.kRight).changeOn {
                if (backModifier.source() > 0.35) {
                    Superstructure.kBackCargoFromLoadingStation.start()
                } else {
                    Superstructure.kFrontCargoFromLoadingStation.start()
                }
            }

            button(kBumperRight).changeOn(Superstructure.kStowedPosition)
        }
        state({ isClimbing }) {
            button(kA).change(autoL3Climb())
            button(kY).change(autoL2Climb())
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