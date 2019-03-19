package org.ghrobotics.frc2019.subsystems

import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.*
import kotlin.math.absoluteValue
import kotlin.math.pow

object Superstructure {

    val heightAboveGround
        get() = Constants.kElevatorHeightFromGround + ElevatorSubsystem.position.meter +
            (Constants.kArmLength * ArmSubsystem.position.sin)

    private val outOfToleranceRange =
        (90.degree - Constants.kArmFlipTolerance)..(90.degree + Constants.kArmFlipTolerance)


    val kFrontHighRocketHatch get() = goToHeightWithAngle(76.inch, 15.degree)
    val kFrontMiddleRocketHatch get() = goToHeightWithAngle(50.inch, 5.degree)

    val kFrontHighRocketCargo get() = goToHeightWithAngle(83.inch, 15.degree)
    val kFrontMiddleRocketCargo get() = goToHeightWithAngle(56.inch, 15.degree)
    val kFrontLowRocketCargo get() = goToHeightWithAngle(26.inch, 15.degree)

    val kFrontHatchFromLoadingStation get() = goToHeightWithAngle(16.inch, 10.degree)
    val kBackHatchFromLoadingStation get() = goToHeightWithAngle(16.inch, 180.degree)

    val kFrontCargoIntake get() = elevatorAndArmHeight(1.inch, (-35).degree)
    val kBackCargoIntake get() = elevatorAndArmHeight(1.inch, (-145).degree)

    val kFrontCargoIntoCargoShip get() = elevatorAndArmHeight(25.inch, 5.degree)
    val kBackCargoFromLoadingStation get() = elevatorAndArmHeight(0.inch, 135.degree)

    val kStowedPosition get() = elevatorAndArmHeight(0.inch, 90.degree)

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

    @Suppress("ComplexMethod")
    fun elevatorAndArmHeight(
        elevatorHeightWanted: Length,
        armAngle: Rotation2d
    ): FalconCommand {
        // Values that store the side of the robot the arm is currently in and the side of the robot that the arm
        // wants to be in.
        val isFrontWanted = armAngle.cos > 0

        // Check if the configuration is valid.
        return sequential {

            // Flip arm vs. don't flip arm.
            +ConditionalCommand(
                {
                    val isFrontCurrent = ArmSubsystem.position.cos > 0
                    isFrontWanted != isFrontCurrent || ArmSubsystem.position in outOfToleranceRange
                },

                // We now need to flip the arm
                sequential {
                    val elevatorLimit = (-2).inch

                    var intakeSafeToOpen = false

                    +parallel {
                        +object : FalconCommand() {
                            init {
                                finishCondition += { intakeSafeToOpen }
                            }

                            override suspend fun execute() {
                                if (IntakeSubsystem.isFullyExtended) {
                                    IntakeSubsystem.wantedExtensionSolenoidState =
                                        IntakeSubsystem.ExtensionSolenoidState.RETRACTED
                                    IntakeSubsystem.wantedLauncherSolenoidState = false
                                }
                            }
                        }
                        // Elevator
                        +sequential {
                            val elevatorWaitCondition = {
                                if (isFrontWanted) {
                                    ArmSubsystem.position <=
                                        90.degree - Constants.kArmFlipTolerance + Constants.kArmClosedLoopTolerance &&
                                        ArmSubsystem.position.cos > 0
                                } else {
                                    ArmSubsystem.position >=
                                        90.degree + Constants.kArmFlipTolerance - Constants.kArmClosedLoopTolerance &&
                                        ArmSubsystem.position.cos < 0
                                }
                            }
                            +sequential {
                                // Zero Elevator
                                +ClosedLoopElevatorCommand((-5).inch)
                                    .overrideExit { ElevatorSubsystem.isZeroed }
                                // Park elevator at 0 after zeroing
                                +ClosedLoopElevatorCommand(elevatorLimit)
                            }.overrideExit(elevatorWaitCondition)
                            // Wait for arm to flip
                            +ConditionCommand(elevatorWaitCondition)

                            // Allow intake to open
                            +InstantRunnableCommand { intakeSafeToOpen = true }

                            +ClosedLoopElevatorCommand(elevatorHeightWanted)
                        }
                        // Arm
                        +sequential {
                            val waitCondition = {
                                ElevatorSubsystem.position < Constants.kElevatorSafeFlipHeight.value
                                    || ElevatorSubsystem.isBottomLimitSwitchPressed
                            }
                            // Prepare arm to flip through elevator
                            +ClosedLoopArmCommand(
                                if (isFrontWanted) {
                                    90.degree + Constants.kArmFlipTolerance
                                } else {
                                    90.degree - Constants.kArmFlipTolerance
                                }
                            ).overrideExit(waitCondition)
                            // Wait for elevator to come down to safe height
                            +ConditionCommand(waitCondition)

                            if (elevatorHeightWanted > Constants.kElevatorSafeFlipHeight + Constants.kElevatorClosedLoopTolerance) {
                                // Consider safe flip if elevator goes up
                                if (isFrontWanted) {
                                    if (armAngle < Constants.kArmSafeFlipAngle) {
                                        //  Use safe flip if it goes near floor
                                        +ClosedLoopArmCommand(Constants.kArmSafeFlipAngle)
                                            .overrideExit { ElevatorSubsystem.position > Constants.kElevatorSafeFlipHeight.value }
                                    }
                                } else {
                                    if (armAngle < 180.degree - Constants.kArmSafeFlipAngle) {
                                        //  Use safe flip if it goes near floor
                                        +ClosedLoopArmCommand(180.degree - Constants.kArmSafeFlipAngle)
                                            .overrideExit { ElevatorSubsystem.position > Constants.kElevatorSafeFlipHeight.value }
                                    }
                                }
                            }

                            +ClosedLoopArmCommand(armAngle)
                        }
                    }
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


