package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.absoluteValue

object DefaultElevatorCommand : FalconCommand(ElevatorSubsystem) {
    override suspend fun initialize() {
        val currentState = ElevatorSubsystem.currentState
        val currentPosition = ElevatorSubsystem.position
        val wantedPosition = if (currentState is ElevatorSubsystem.ElevatorState.SetPointState
            && (currentState.position - currentPosition).absoluteValue <= Constants.kElevatorClosedLoopTolerance.value
        ) {
            currentState.position
        } else {
            currentPosition
        }
        ElevatorSubsystem.wantedState = ElevatorSubsystem.ElevatorState.Position(wantedPosition)
    }
}