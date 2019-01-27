package org.ghrobotics.frc2019.subsystems

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

object Superstructure {

    val heightAboveGround
        get() = Constants.kElevatorHeightFromGround + ElevatorSubsystem.elevatorPosition +
            (Constants.kArmLength * ArmSubsystem.armPosition.sin)


//    val kFrontHighRocketHatch get() = goToHeightWithAngle(75.inch, 0.degree)
//    val kFrontHighRocketCargo get() = goToHeightWithAngle(84.inch, 45.degree)
//    val kFrontMiddleRocketHatch get() = goToHeightWithAngle(47.inch, 0.degree)
//    val kBackLoadingStation get() = goToHeightWithAngle(20.inch, 180.degree)
//    val kFrontLoadingStation get() = goToHeightWithAngle(20.inch, 0.degree)

    val kFrontHighRocketHatch
        get() = ClosedLoopElevatorCommand(
            75.inch - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft
        )

    val kFrontHighRocketCargo get() = ClosedLoopElevatorCommand(65.inch)

    val kFrontMiddleRocketHatch
        get() = ClosedLoopElevatorCommand(
            47.inch - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft
        )

    val kBackLoadingStation
        get() = ClosedLoopElevatorCommand(
            20.inch - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft
        )

    val kFrontLoadingStation
        get() = ClosedLoopElevatorCommand(
            20.inch - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft
        )

    fun goToHeightWithAngle(
        heightAboveGround: Length,
        armAngle: Rotation2d
    ) = sequential {

        require(armAngle !in (90.degree - Constants.kArmFlipTolerance)..(90.degree + Constants.kArmFlipTolerance))

        val elevatorHeightWanted =
            (heightAboveGround - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft -
                (Constants.kArmLength * armAngle.sin)).coerceIn(0.inch, Constants.kMaxElevatorHeightFromZero)

        val isFrontWanted = armAngle < 90.degree
        val isFrontCurrent = ArmSubsystem.armPosition < 90.degree

        +ConditionalCommand(
            IntakeSubsystem.isFullyExtended,
            InstantRunnableCommand { IntakeSubsystem.extensionSolenoid.set(false) })

        +ConditionalCommand(
            { isFrontWanted != isFrontCurrent },
            sequential {
                val zeroElevator = ClosedLoopElevatorCommand(0.inch)

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

                +parallel {
                    +ClosedLoopArmCommand(armAngle)
                    +sequential {
                        +ConditionCommand {
                            if (isFrontWanted) {
                                ArmSubsystem.armPosition < 90.degree - Constants.kArmFlipTolerance
                            } else {
                                ArmSubsystem.armPosition > 90.degree + Constants.kArmFlipTolerance
                            }
                        }
                        +ClosedLoopElevatorCommand(elevatorHeightWanted)
                    }
                }
            },
            parallel {
                +ClosedLoopElevatorCommand(elevatorHeightWanted)
                +ClosedLoopArmCommand(armAngle)
            }
        )
    }
}
