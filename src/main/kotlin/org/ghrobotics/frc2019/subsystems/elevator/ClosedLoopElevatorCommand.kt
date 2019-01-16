package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length

class ClosedLoopElevatorCommand(private val target: Length) : FalconCommand(ElevatorSubsystem) {
    
    init {
        finishCondition += {
            (ElevatorSubsystem.elevatorPosition - target).absoluteValue < Constants.kElevatorClosedLoopTolerance
        }
    }

    override suspend fun initialize() {
        ElevatorSubsystem.elevatorPosition = target
    }
}