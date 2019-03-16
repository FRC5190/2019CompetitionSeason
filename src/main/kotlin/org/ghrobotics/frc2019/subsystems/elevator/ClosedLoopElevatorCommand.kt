package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length
import kotlin.math.absoluteValue

class ClosedLoopElevatorCommand(private val target: Length) : FalconCommand(ElevatorSubsystem) {

    init {
        finishCondition += {
            (ElevatorSubsystem.position - target.value).absoluteValue < Constants.kElevatorClosedLoopTolerance.value
                && ElevatorSubsystem.velocity < Constants.kElevatorClosedLoopVelocityTolerance.value
        }
    }

    override suspend fun initialize() {
        ElevatorSubsystem.wantedState = ElevatorSubsystem.ElevatorState.MotionMagic(target.value)
    }
}