package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.absoluteValue

object DefaultArmCommand : FalconCommand(ArmSubsystem) {
    override suspend fun initialize() {
        val currentState = ArmSubsystem.currentState
        val currentPosition = ArmSubsystem.position
        val wantedPosition = if (currentState is ArmSubsystem.ArmState.SetPointState
            && (currentState.position - currentPosition).value.absoluteValue <= Constants.kElevatorClosedLoopTolerance.value
        ) {
            currentState.position
        } else {
            currentPosition
        }
        ArmSubsystem.wantedState = ArmSubsystem.ArmState.Position(wantedPosition)
    }
}