package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.PeriodicRunnableCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.utils.Source

object TuneElevatorRoutines {

    private const val voltageStepUp = 0.05 / 50.0
    private const val voltageStepDown = 0.75 / 50.0

    val tuneKgRoutine
        get() = sequential {
            +Superstructure.kFrontHatchFromLoadingStation

            var goingUpBeforeSwitchKg = 0.0
            var goingDownBeforeSwitchKg = 0.0

            var goingUpAfterSwitchKg = 0.0
            var goingDownAfterSwitchKg = 0.0

            +InstantRunnableCommand {
                goingUpBeforeSwitchKg = 0.0
                goingDownBeforeSwitchKg = 0.0
                goingUpAfterSwitchKg = 0.0
                goingDownAfterSwitchKg = 0.0
            }
            // BEFORE SWITCH Kg that allows elevator to go up
            +ClosedLoopElevatorCommand(Constants.kElevatorSwitchHeight / 2.0)
            +parallel {
                var voltageOutput = 0.0
                +InstantRunnableCommand { voltageOutput = ElevatorSubsystem.voltage }
                +PeriodicRunnableCommand({
                    voltageOutput += voltageStepUp
                    goingUpBeforeSwitchKg = voltageOutput / Constants.kAccelerationDueToGravity / 12.0
                }, Source(false))
                +OpenLoopElevatorCommand { voltageOutput / 12.0 }
            }.withExit { ElevatorSubsystem.velocity.value > 0.5 }

            // BEFORE SWITCH Kg that allows elevator to go down
            +ClosedLoopElevatorCommand(Constants.kElevatorSwitchHeight / 2.0)
            +parallel {
                var voltageOutput = 0.0
                +InstantRunnableCommand {
                    voltageOutput = Math.max(
                        ElevatorSubsystem.voltage,
                        goingUpBeforeSwitchKg * Constants.kAccelerationDueToGravity
                    )
                }
                +PeriodicRunnableCommand({
                    voltageOutput -= voltageStepDown
                    goingDownBeforeSwitchKg = voltageOutput / Constants.kAccelerationDueToGravity / 12.0
                }, Source(false))
                +OpenLoopElevatorCommand { voltageOutput / 12.0 }
            }.withExit { ElevatorSubsystem.velocity.value < -0.5 }


            // AFTER SWITCH Kg that allows elevator to go up
            +ClosedLoopElevatorCommand((Constants.kElevatorSwitchHeight + Constants.kMaxElevatorHeightFromZero) / 2.0)
            +parallel {
                var voltageOutput = 0.0
                +InstantRunnableCommand { voltageOutput = ElevatorSubsystem.voltage }
                +PeriodicRunnableCommand({
                    voltageOutput += voltageStepUp
                    goingUpAfterSwitchKg = voltageOutput / Constants.kAccelerationDueToGravity / 12.0
                }, Source(false))
                +OpenLoopElevatorCommand { voltageOutput / 12.0 }
            }.withExit { ElevatorSubsystem.velocity.value > 0.5 }

            // BEFORE SWITCH Kg that allows elevator to go down
            +ClosedLoopElevatorCommand((Constants.kElevatorSwitchHeight + Constants.kMaxElevatorHeightFromZero) / 2.0)
            +parallel {
                var voltageOutput = 0.0
                +InstantRunnableCommand {
                    voltageOutput = Math.max(
                        ElevatorSubsystem.voltage,
                        goingUpAfterSwitchKg * Constants.kAccelerationDueToGravity
                    )
                }
                +PeriodicRunnableCommand({
                    voltageOutput -= voltageStepDown
                    goingDownAfterSwitchKg = voltageOutput / Constants.kAccelerationDueToGravity / 12.0
                }, Source(false))
                +OpenLoopElevatorCommand { voltageOutput / 12.0 }
            }.withExit { ElevatorSubsystem.velocity.value < -0.5 }

            +InstantRunnableCommand {
                val actualBeforeSwitchKg = (goingUpBeforeSwitchKg + goingDownBeforeSwitchKg) / 2.0
                val actualBeforeSwitchKs = goingUpBeforeSwitchKg - actualBeforeSwitchKg

                val actualAfterSwitchKg = (goingUpAfterSwitchKg + goingDownAfterSwitchKg) / 2.0
                val actualAfterSwitchKs = goingUpAfterSwitchKg - actualAfterSwitchKg

                println(
                    "Found Elevator Constants:\n" +
                        "Before Switch Kg: $actualBeforeSwitchKg Ks: $actualBeforeSwitchKs\n" +
                        "After Switch Kg: $actualAfterSwitchKg Ks: $actualAfterSwitchKs"
                )
            }
        }

}