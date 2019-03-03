package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length

class ClosedLoopElevatorCommand(private val target: Length) : FalconCommand(ElevatorSubsystem) {

    init {
        finishCondition += {
            (ElevatorSubsystem.position - target).absoluteValue < Constants.kElevatorClosedLoopTolerance &&
                ElevatorSubsystem.velocity < Constants.kElevatorClosedLoopVelocityTolerance
        }
    }

    override suspend fun initialize() {
        ElevatorSubsystem.setPosition(target)
    }
}