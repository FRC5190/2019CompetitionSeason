package org.ghrobotics.frc2019.subsystems

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.commands.ConditionalCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

object Superstructure {

    val kFrontHighRocketHatch get() = goToHeightWithAngle(76.inch, 0.degree)
    val kFrontHighRocketCargo get() = goToHeightWithAngle(80.inch, 45.degree)
    val kBackLoadingStation get() = goToHeightWithAngle(21.inch, 180.degree)
    val kFrontLoadingStation get() = goToHeightWithAngle(21.inch, 0.degree)

    fun goToHeightWithAngle(
        heightAboveGround: Length,
        armAngle: Rotation2d
    ) = sequential {

        require(armAngle !in 85.degree..95.degree)

        val elevatorHeightWanted = (heightAboveGround - Constants.kElevatorHeightFromGround -
            (Constants.kArmLength * armAngle.sin)).coerceIn(0.inch, Constants.kMaxElevatorHeightFromZero)

        val isFrontWanted = armAngle < 90.degree
        val isFrontCurrent = ArmSubsystem.armPosition < 90.degree

        +ConditionalCommand(
            { isFrontWanted != isFrontCurrent },
            sequential {
                val zeroElevator = ClosedLoopElevatorCommand(0.inch)

                +parallel {
                    +zeroElevator
                    +ClosedLoopArmCommand(if (isFrontWanted) 95.degree else 85.degree)
                }.withExit { zeroElevator.wrappedValue.isCompleted }

                +parallel {
                    +ClosedLoopArmCommand(armAngle)
                    +sequential {
                        +ConditionCommand {
                            if (isFrontWanted) ArmSubsystem.armPosition < 85.degree else ArmSubsystem.armPosition > 95.degree
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
