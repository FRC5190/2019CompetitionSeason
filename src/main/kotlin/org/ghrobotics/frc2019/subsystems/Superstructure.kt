package org.ghrobotics.frc2019.subsystems

import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.Source

object Superstructure {

    val heightAboveGround
        get() = Constants.kElevatorHeightFromGround + ElevatorSubsystem.elevatorPosition +
            (Constants.kArmLength * ArmSubsystem.armPosition.sin)


    val kFrontHighRocketHatch get() = goToHeightWithAngle(75.inch, 0.degree)
    val kFrontHighRocketCargo get() = goToHeightWithAngle(84.inch, 45.degree)
    val kFrontMiddleRocketHatch get() = goToHeightWithAngle(47.inch, 0.degree)
    val kBackLoadingStation get() = goToHeightWithAngle(20.inch, 180.degree)
    val kFrontLoadingStation get() = goToHeightWithAngle(20.inch, 0.degree)

    private fun goToHeightWithAngle(
        heightAboveGround: Length,
        armAngle: Rotation2d
    ): FalconCommand {

        // Calculates the wanted elevator height.
        val elevatorHeightWanted =
            (heightAboveGround - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft -
                (Constants.kArmLength * armAngle.sin)).coerceIn(0.inch, Constants.kMaxElevatorHeightFromZero)

        // Values that store the side of the robot the arm is currently in and the side of the robot that the arm
        // wants to be in.
        val isFrontWanted = armAngle < 90.degree

        // Check if the configuration is valid.
        return ConditionalCommand(Source(checkIfConfigValid(heightAboveGround, armAngle)), sequential {

            // Closes the intake.
            +ConditionalCommand(
                IntakeSubsystem.isFullyExtended,
                InstantRunnableCommand { IntakeSubsystem.extensionSolenoid.set(false) })

            // Flip arm vs. don't flip arm.
            +ConditionalCommand(
                {
                    val isFrontCurrent = ArmSubsystem.armPosition < 90.degree
                    isFrontWanted != isFrontCurrent
                },

                // We now need to flip the arm
                sequential {
                    // Zero the elevator
                    val zeroElevator = ClosedLoopElevatorCommand(0.inch)

                    // Bring elevator down while moving the arm to a position where it is safe.
                    +parallel {
                        +zeroElevator
                        +ClosedLoopArmCommand(
                            if (isFrontWanted) {
                                90.degree + Constants.kArmFlipTolerance
                            } else {
                                90.degree - Constants.kArmFlipTolerance
                            }
                        )
                    }.overrideExit { ElevatorSubsystem.isBottomLimitSwitchPressed }

                    // Flip the arm. Take the elevator up to final position once the arm is out of the way.
                    +parallel {
                        +ClosedLoopArmCommand(armAngle)
                        +sequential {
                            +ConditionCommand {
                                if (isFrontWanted) {
                                    ArmSubsystem.armPosition <= 90.degree - Constants.kArmFlipTolerance
                                } else {
                                    ArmSubsystem.armPosition >= 90.degree + Constants.kArmFlipTolerance
                                }
                            }
                            +ClosedLoopElevatorCommand(elevatorHeightWanted)
                        }
                    }
                },

                // We don't need to flip the arm. Take the elevator and arm to desired locations.
                parallel {
                    +ClosedLoopElevatorCommand(elevatorHeightWanted)
                    +ClosedLoopArmCommand(armAngle)
                }
            )
        }, InstantRunnableCommand { DriverStation.reportError("Desired Superstructure State is Invalid.", false) })
    }

    private fun checkIfConfigValid(heightAboveGround: Length, armAngle: Rotation2d) =
        (armAngle in (90.degree - Constants.kArmFlipTolerance)..(90.degree + Constants.kArmFlipTolerance)) ||
            (armAngle > 90.degree
                && heightAboveGround + Constants.kIntakeCradleHeight <= Constants.kElevatorCrossbarHeightFromGround)

    private fun enforceSoftLimits(armPosition: Rotation2d) {
        // TODO
    }
}


