package org.ghrobotics.frc2019.subsystems

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

class SuperstructureCommand(
    heightAboveGround: Length,
    private val armAngle: Rotation2d
) : FalconCommand(ElevatorSubsystem, ArmSubsystem) {

    private var currentState = State.GO_TO_HEIGHT
    private val elevatorHeightWanted = (heightAboveGround - Constants.kElevatorHeightFromGround -
        (Constants.kArmLength * armAngle.sin)).coerceIn(0.inch, Constants.kMaxElevatorHeightFromZero)

    init {
        require(armAngle.degree !in 85.0..95.0)
        require(armAngle.degree in 0.0..180.0)
        finishCondition += {
            (ElevatorSubsystem.elevatorPosition - elevatorHeightWanted).absoluteValue < Constants.kElevatorClosedLoopTolerance
                && (ArmSubsystem.armPosition - armAngle).absoluteValue < Constants.kArmClosedLoopTolerance
        }
    }

    override suspend fun initialize() {
        val isFrontWanted = armAngle < 90.degree
        val isFrontCurrent = ArmSubsystem.armPosition < 90.degree
        currentState = when {
            isFrontWanted != isFrontCurrent -> State.FLIP_ARM_ELEVATOR_DOWN
            else -> State.GO_TO_HEIGHT
        }
    }

    override suspend fun execute() {
        when (currentState) {
            State.FLIP_ARM_ELEVATOR_DOWN -> {
                ElevatorSubsystem.elevatorPosition = 0.inch
                if (ElevatorSubsystem.elevatorPosition < 2.inch) {
                    currentState = State.FLIP_ARM
                }
            }
            State.FLIP_ARM -> {
                ArmSubsystem.armPosition = armAngle
                val isFrontWanted = armAngle < 90.degree
                if (isFrontWanted) {
                    if (ArmSubsystem.armPosition < 85.degree) {
                        currentState = State.GO_TO_HEIGHT
                    }
                } else {
                    if (ArmSubsystem.armPosition > 95.degree) {
                        currentState = State.GO_TO_HEIGHT
                    }
                }
            }
            State.GO_TO_HEIGHT -> ElevatorSubsystem.elevatorPosition = elevatorHeightWanted
        }
    }

    private enum class State {
        FLIP_ARM_ELEVATOR_DOWN,
        FLIP_ARM,
        GO_TO_HEIGHT
    }

    companion object {
        val kFrontHighRocketHatch = SuperstructureCommand(76.inch, 0.degree)
        val kFrontHighRocketCargo = SuperstructureCommand(80.inch, 45.degree)
        val kBackLoadingStation = SuperstructureCommand(21.inch, 180.degree)
        val kFrontLoadingStation = SuperstructureCommand(21.inch, 0.degree)
    }
}
