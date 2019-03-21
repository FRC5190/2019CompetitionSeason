package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.PeriodicRunnableCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.max
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.Source

object TuneArmRoutines {

    private const val voltageStepUp = 0.05 / 50.0
    private const val voltageStepDown = 0.75 / 50.0

    val tuneKgRoutine
        get() = sequential {
            +Superstructure.kFrontHatchFromLoadingStation

            val deltaArea = 10.degree

            var goingUpKg = 0.0
            var goingDownKg = 0.0

            +InstantRunnableCommand {
                goingUpKg = 0.0
                goingDownKg = 0.0
            }
            // Find the Kg that allows the arm to start going up
            +ClosedLoopArmCommand(-deltaArea)
            +parallel {
                var voltageOutput = 0.0
                var isMoving = false
                +InstantRunnableCommand {
                    voltageOutput = ArmSubsystem.voltage
                    isMoving = false
                }
                +PeriodicRunnableCommand({
                    if (ArmSubsystem.velocity.value < 0.25) {
                        isMoving = false
                        voltageOutput += voltageStepUp
                    } else if (!isMoving) {
                        isMoving = true
                        goingUpKg =
                            voltageOutput / ArmSubsystem.position.cos / Constants.kAccelerationDueToGravity / 12.0
                    }
                }, Source(false))
                +OpenLoopArmCommand { voltageOutput / 12.0 }
            }.withExit { ArmSubsystem.position.sin > 0 }

            // Find the Kg that allows the arm to start going down
            +ClosedLoopArmCommand(deltaArea)
            +parallel {
                var voltageOutput = 0.0
                var isMoving = false
                +InstantRunnableCommand {
                    voltageOutput = Math.max(
                        ArmSubsystem.voltage,
                        goingUpKg * ArmSubsystem.position.cos * Constants.kAccelerationDueToGravity
                    )
                    isMoving = false
                }
                +PeriodicRunnableCommand({
                    if (ArmSubsystem.velocity.value > -0.25) {
                        isMoving = false
                        voltageOutput -= voltageStepDown
                    } else if (!isMoving) {
                        isMoving = true
                        goingDownKg =
                            voltageOutput / ArmSubsystem.position.cos / Constants.kAccelerationDueToGravity / 12.0
                    }
                }, Source(false))
                +OpenLoopArmCommand { voltageOutput / 12.0 }
            }.withExit { ArmSubsystem.position.sin < 0 }

            +InstantRunnableCommand {
                val actualKg = (goingUpKg + goingDownKg) / 2.0
                val actualKs = goingUpKg - actualKg

                println("Found Arm Kg: $actualKg Ks: $actualKs")
            }
        }

    val findMaxVelocity
        get() = sequential {
            +Superstructure.kFrontCargoIntake
            +ClosedLoopElevatorCommand(0.inch).withExit { ElevatorSubsystem.isBottomLimitSwitchPressed }

            var maxVelocity = 0.degree.velocity
            var maxVelocityAt90 = 0.degree.velocity

            +InstantRunnableCommand {
                maxVelocity = 0.degree.velocity
                maxVelocityAt90 = 0.degree.velocity
            }

            +parallel {
                +PeriodicRunnableCommand({
                    maxVelocity = max(ArmSubsystem.velocity, maxVelocity)
                    if (ArmSubsystem.position.cos >= 0) {
                        maxVelocityAt90 = maxVelocity
                    }
                }, Source(true))
                +OpenLoopArmCommand(100.0)
            }.withExit {
                val position = ArmSubsystem.position
                position.cos < 0 && position.sin < 0
            }
            +ClosedLoopArmCommand(ArmSubsystem.position)

            +InstantRunnableCommand {
                println("Arm had a max velocity of ${maxVelocity.value} rad/s and a max velocity at 90 of ${maxVelocityAt90.value} rad/s")
            }
        }


}