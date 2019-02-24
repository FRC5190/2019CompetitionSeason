package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand

class BringUpBackCommand : FalconCommand(ClimbSubsystem) {
    init {
        finishCondition += { ClimbSubsystem.backWinchMaster.sensorCollection.isRevLimitSwitchClosed }
    }

    override suspend fun execute() {
        if (ClimbSubsystem.rawBack > 100) {
            ClimbSubsystem.backWinchPercentOutput = -1.0
        } else {
            ClimbSubsystem.backWinchPercentOutput = -0.5
        }
    }
}