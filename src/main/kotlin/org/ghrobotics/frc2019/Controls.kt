/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.OpenLoopArmCommand
import org.ghrobotics.frc2019.subsystems.climb.AutoClimbRoutines
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.TuneElevatorRoutines
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.pow
import kotlin.math.withSign

object Controls {

    var isClimbing = false
        private set

    val driverFalconXbox = xboxController(0) {
        registerEmergencyMode()

//        pov(90).changeOn(TuneElevatorRoutines.tuneKgRoutine)
        pov(270).changeOn { IntakeSubsystem.badIntakeOffset += .25.inch }
        pov(90).changeOn { IntakeSubsystem.badIntakeOffset -= .25.inch }

        state({ !isClimbing }) {
            // Vision align
            button(kY).change(VisionDriveCommand(VisionDriveCommand.TargetSide.FRONT))
            button(kB).change(VisionDriveCommand(VisionDriveCommand.TargetSide.BACK))

            // Shifting
            button(kA).changeOn { DriveSubsystem.lowGear = true }
            button(kA).changeOff { DriveSubsystem.lowGear = false }

            // Intake
            triggerAxisButton(GenericHID.Hand.kLeft).change(IntakeHatchCommand(true))
            button(kBumperLeft).change(IntakeHatchCommand(false))

            triggerAxisButton(GenericHID.Hand.kRight).change(IntakeCargoCommand(true))
            button(kBumperRight).change(IntakeCargoCommand(false))
        }
    }

    var backModifier = false
        private set

    val operatorFalconXbox = xboxController(1) {
        registerEmergencyMode()

        // Climb
        button(kB).changeOn {
            isClimbing = !isClimbing
            DriveSubsystem.lowGear = true
            Superstructure.kStowedPosition.start()
        }

        state({ !isClimbing }) {

            /** MANUAL CONTROL **/
            axisButton(1, 0.1) { change(OpenLoopElevatorCommand(source.map { it.pow(2).withSign(-it) * .5 })) }
            axisButton(5, 0.1) { change(OpenLoopArmCommand(source.map { it.pow(2).withSign(-it) * .5 })) }

            /** PRESETS **/
            triggerAxisButton(GenericHID.Hand.kLeft, 0.20) {
                changeOn { backModifier = true }
                changeOff { backModifier = false }
            }

            pov(0).changeOn {
                if (IntakeSubsystem.isHoldingCargo) Superstructure.kFrontHighRocketCargo.start()
                else Superstructure.kFrontHighRocketHatch.start()
            }
            pov(90).changeOn {
                if (IntakeSubsystem.isHoldingCargo) Superstructure.kFrontMiddleRocketCargo.start()
                else Superstructure.kFrontMiddleRocketHatch.start()
            }
            pov(180).changeOn {
                when {
                    backModifier -> Superstructure.kBackHatchFromLoadingStation.start()
                    IntakeSubsystem.isHoldingCargo -> Superstructure.kFrontLowRocketCargo.start()
                    else -> Superstructure.kFrontHatchFromLoadingStation.start()
                }
            }
            pov(270).changeOn {
                if (backModifier) Superstructure.kBackCargoIntake.start()
                else Superstructure.kFrontCargoIntake.start()
            }
            triggerAxisButton(GenericHID.Hand.kRight).changeOn {
                if (backModifier) Superstructure.kBackCargoFromLoadingStation.start()
                else Superstructure.kFrontCargoIntoCargoShip.start()
            }
            button(kBumperRight).changeOn(Superstructure.kStowedPosition)
        }

        state({ isClimbing }) {
            button(kA).change(AutoClimbRoutines.autoL3Climb)
            button(kY).change(AutoClimbRoutines.autoL2Climb)
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
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}