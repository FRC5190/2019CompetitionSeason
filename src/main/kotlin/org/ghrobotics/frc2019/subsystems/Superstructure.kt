package org.ghrobotics.frc2019.subsystems

import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.*
import kotlin.math.absoluteValue
import kotlin.math.pow

object Superstructure {

    val heightAboveGround
        get() = Constants.kElevatorHeightFromGround + ElevatorSubsystem.elevatorPosition +
            (Constants.kArmLength * ArmSubsystem.armPosition.sin)

    private val outOfToleranceRange =
        (90.degree - Constants.kArmFlipTolerance)..(90.degree + Constants.kArmFlipTolerance)


    val kFrontHighRocketHatch get() = goToHeightWithAngle(76.inch, 15.degree)
    val kFrontMiddleRocketHatch get() = goToHeightWithAngle(50.inch, 5.degree)

    val kFrontHighRocketCargo get() = goToHeightWithAngle(83.inch, 45.degree)
    val kFrontMiddleRocketCargo get() = goToHeightWithAngle(56.inch, 45.degree)
    val kFrontLowRocketCargo get() = goToHeightWithAngle(32.inch, 45.degree)
    val kBackLowRocketCargo get() = goToHeightWithAngle(32.inch, 135.degree)

    val kFrontHatchFromLoadingStation get() = goToHeightWithAngle(20.inch, 0.degree)
    val kBackHatchFromLoadingStation get() = goToHeightWithAngle(20.inch, 180.degree)

    val kFrontCargoIntake get() = elevatorAndArmHeight(0.inch, (-20).degree)
    val kBackCargoIntake get() = elevatorAndArmHeight(0.inch, (-160).degree)

    val kFrontCargoFromLoadingStation get() = elevatorAndArmHeight(0.inch, 45.degree)
    val kBackCargoFromLoadingStation get() = elevatorAndArmHeight(0.inch, 135.degree)

    private fun goToHeightWithAngle(
        heightAboveGround: Length,
        armAngle: Rotation2d
    ): FalconCommand {

        if (!checkIfConfigValid(heightAboveGround, armAngle)) {
            return InstantRunnableCommand {
                DriverStation.reportError(
                    "Desired Superstructure State is Invalid." +
                        "\nheightAboveGround: ${heightAboveGround.inch}inch" +
                        "\narmAngle: ${armAngle.degree}degree",
                    false
                )
            }
        }

        // Calculates the wanted elevator height.
        val elevatorHeightWanted =
            (heightAboveGround - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft -
                (Constants.kArmLength * armAngle.sin)).coerceIn(0.inch, Constants.kMaxElevatorHeightFromZero)

        return elevatorAndArmHeight(elevatorHeightWanted, armAngle)
    }

    private fun elevatorAndArmHeight(
        elevatorHeightWanted: Length,
        armAngle: Rotation2d
    ): FalconCommand {
        // Values that store the side of the robot the arm is currently in and the side of the robot that the arm
        // wants to be in.
        val isFrontWanted = armAngle.cos >= 0

        // Check if the configuration is valid.
        return sequential {

            // Flip arm vs. don't flip arm.
            +ConditionalCommand(
                {
                    val isFrontCurrent = ArmSubsystem.armPosition.cos >= 0
                    isFrontWanted != isFrontCurrent || ArmSubsystem.armPosition in outOfToleranceRange
                },

                // We now need to flip the arm
                sequential {
                    +InstantRunnableCommand { println("FLIPPING") }

//                    +ConditionalCommand(
//                        IntakeSubsystem.isFullyExtended,
//                        IntakeCloseCommand()
//                    )

                    val elevatorLimit = (-2).inch

                    // Bring elevator down while moving the arm to a position where it is safe.
                    +parallel {
                        +sequential {
                            // Zero Elevator
                            +ClosedLoopElevatorCommand((-5).inch)
                                .overrideExit { ElevatorSubsystem.isZeroed }
                            // Park elevator at 0 after zeroing
                            +ClosedLoopElevatorCommand(elevatorLimit)
                        }
                        // Prepare arm to flip through elevator
                        +ClosedLoopArmCommand(
                            if (isFrontWanted) {
                                90.degree + Constants.kArmFlipTolerance
                            } else {
                                90.degree - Constants.kArmFlipTolerance
                            }
                        )
                    }.overrideExit {
                        // Exit when the elevator is zeroed and within tolerance of zero
                        ElevatorSubsystem.isZeroed &&
                            ElevatorSubsystem.elevatorPosition.absoluteValue < Constants.kElevatorClosedLoopTolerance &&
                            ElevatorSubsystem.velocity < Constants.kElevatorClosedLoopVelocityTolerance
                    }

                    // Ensure elevator is parked at 0 and not -5
                    +ClosedLoopElevatorCommand(elevatorLimit)
                        .overrideExit { ElevatorSubsystem.isZeroed }

                    // Flip arm but angle up while elevator is going up
                    +parallel {
                        val safeFlipAngle = if (isFrontWanted) {
                            90.degree - Constants.kArmFlipTolerance - Constants.kArmClosedLoopTolerance / 2.0
                        } else {
                            90.degree + Constants.kArmFlipTolerance + Constants.kArmClosedLoopTolerance / 2.0
                        }

                        +sequential {
                            if(elevatorHeightWanted > Constants.kElevatorSafeFlipHeight + Constants.kElevatorClosedLoopTolerance) {
                                +ClosedLoopArmCommand(safeFlipAngle)
                                    .overrideExit { ElevatorSubsystem.elevatorPosition > Constants.kElevatorSafeFlipHeight }
                            }
                            +ClosedLoopArmCommand(armAngle)
                        }

                        +sequential {
                            +ConditionCommand {
                                if (isFrontWanted) {
                                    ArmSubsystem.armPosition <=
                                        90.degree - Constants.kArmFlipTolerance + Constants.kArmClosedLoopTolerance &&
                                        ArmSubsystem.armPosition.cos > 0
                                } else {
                                    ArmSubsystem.armPosition >=
                                        90.degree + Constants.kArmFlipTolerance - Constants.kArmClosedLoopTolerance &&
                                        ArmSubsystem.armPosition.cos < 0
                                }
                            }
                            +ClosedLoopElevatorCommand(elevatorHeightWanted)
                        }
                    }

//                    // Flip the arm. Take the elevator up to final position once the arm is out of the way.
//                    +parallel {
//                        +ClosedLoopArmCommand(armAngle)
//                        +sequential {
//                            +ConditionCommand {
//                                if (isFrontWanted) {
//                                    ArmSubsystem.armPosition <= 90.degree - Constants.kArmFlipTolerance &&
//                                        ArmSubsystem.armPosition.cos > 0
//                                } else {
//                                    ArmSubsystem.armPosition >= 90.degree + Constants.kArmFlipTolerance &&
//                                        ArmSubsystem.armPosition.cos < 0
//                                }
//                            }
//                            +ClosedLoopElevatorCommand(elevatorHeightWanted)
//                        }
//                    }
                },

                // We don't need to flip the arm. Take the elevator and arm to desired locations.
                parallel {
                    +ClosedLoopElevatorCommand(elevatorHeightWanted)
                    +ClosedLoopArmCommand(armAngle)
                }
            )
        }
    }

    private fun checkIfConfigValid(heightAboveGround: Length, armAngle: Rotation2d) =
        (armAngle !in outOfToleranceRange) ||
            (armAngle > 90.degree
                && heightAboveGround + Constants.kIntakeCradleHeight <= Constants.kElevatorCrossbarHeightFromGround)

    // MATH SHOULD WORK (its messy because I'm too lazy to simplify)


    fun calcDurationOfArm(
        currentAngle: Rotation2d,
        finalAngle: Rotation2d
    ): Time {
        // Starts and ends at rest

        val x = (currentAngle - finalAngle).value.absoluteValue
        val maxTriangleVelocity = Math.sqrt(Constants.kArmAcceleration.value * x)
        return when {
            maxTriangleVelocity > Constants.kArmCruiseVelocity.value -> {
                val distanceWhenAccel = Constants.kArmCruiseVelocity.value.pow(2.0) /
                    (2.0 * Constants.kArmAcceleration.value)
                Time(
                    Constants.kArmCruiseVelocity.value / Constants.kArmAcceleration.value * 2 +
                        (x - distanceWhenAccel * 2.0) / Constants.kArmCruiseVelocity.value
                )
            }
            else -> Time(maxTriangleVelocity / Constants.kArmAcceleration.value * 2)
        }
    }

    fun calcLevelOutArmHeight(
        armDuration: Time
    ): Length {
        // Currently cruise but ends at rest

        val maxTriangleVelocity = armDuration.value * Constants.kElevatorAcceleration.value
        return when {
            maxTriangleVelocity > Constants.kElevatorCruiseVelocity.value -> {
                val timeToAccel = (Constants.kElevatorCruiseVelocity.value / Constants.kElevatorAcceleration.value)
                Length(
                    (0.5 * Constants.kElevatorAcceleration.value * timeToAccel.pow(2.0)) +
                        (armDuration.value - timeToAccel) * Constants.kElevatorCruiseVelocity.value
                )
            }
            else -> Length(maxTriangleVelocity.pow(2.0) / (2.0 * Constants.kElevatorAcceleration.value))
        }
    }

    private fun enforceSoftLimits(armPosition: Rotation2d) {
        // TODO
    }
}


