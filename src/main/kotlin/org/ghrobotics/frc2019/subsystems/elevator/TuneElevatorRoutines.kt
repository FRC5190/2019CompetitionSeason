package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.PeriodicRunnableCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derivedunits.inchesPerSecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

object TuneElevatorRoutines {

    private const val voltageStepUp = 0.3 / 50.0
    private const val voltageStepDown = 0.3 / 50.0

    private const val movementVelocity = 1.0

    val tuneKgRoutine
        get() = sequential {
            +Superstructure.kFrontHatchFromLoadingStation.withTimeout(5.second)

            var goingUpBeforeSwitchKg = 0.0
            var goingDownBeforeSwitchKg = 0.0

            var goingUpAfterSwitchKg = 0.0
            var goingDownAfterSwitchKg = 0.0

            // BEFORE SWITCH Kg that allows elevator to go up
            +ClosedLoopElevatorCommand(Constants.kElevatorSwitchHeight / 2.0)
            +InstantRunnableCommand { goingUpBeforeSwitchKg = ElevatorSubsystem.voltage }
            +parallel {
                +PeriodicRunnableCommand({
                    goingUpBeforeSwitchKg += voltageStepUp
                }, Source(false))
                +OpenLoopElevatorCommand(false) { goingUpBeforeSwitchKg / 12.0 }
            }.withExit {
                ElevatorSubsystem.velocity.inchesPerSecond > movementVelocity
            }

            // BEFORE SWITCH Kg that allows elevator to go down
            +InstantRunnableCommand { goingDownBeforeSwitchKg = goingUpBeforeSwitchKg }
            +parallel {
                +PeriodicRunnableCommand({
                    goingDownBeforeSwitchKg -= voltageStepDown
                }, Source(false))
                +OpenLoopElevatorCommand(false) { goingDownBeforeSwitchKg / 12.0 }
            }.withExit {
                ElevatorSubsystem.velocity.inchesPerSecond < -movementVelocity
            }


            // AFTER SWITCH Kg that allows elevator to go up
            +ClosedLoopElevatorCommand((Constants.kElevatorSwitchHeight + Constants.kMaxElevatorHeightFromZero) / 2.0)
            +InstantRunnableCommand { goingUpAfterSwitchKg = ElevatorSubsystem.voltage }
            +parallel {
                +PeriodicRunnableCommand({
                    goingUpAfterSwitchKg += voltageStepUp
                }, Source(false))
                +OpenLoopElevatorCommand(false) { goingUpAfterSwitchKg / 12.0 }
            }.withExit {
                ElevatorSubsystem.velocity.inchesPerSecond > movementVelocity
            }

            // BEFORE SWITCH Kg that allows elevator to go down
            +InstantRunnableCommand { goingDownAfterSwitchKg = goingUpAfterSwitchKg }
            +parallel {
                +PeriodicRunnableCommand({
                    goingDownAfterSwitchKg -= voltageStepDown
                }, Source(false))
                +OpenLoopElevatorCommand(false) { goingDownAfterSwitchKg / 12.0 }
            }.withExit {
                ElevatorSubsystem.velocity.inchesPerSecond < -movementVelocity
            }

            +InstantRunnableCommand {
                println("CALCULATING")

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