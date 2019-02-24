package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand

class BringUpFrontCommand : FalconCommand(ClimbSubsystem) {
    init {
        finishCondition += { ClimbSubsystem.frontWinchMaster.sensorCollection.isRevLimitSwitchClosed }
    }

    override suspend fun execute() {
        if (ClimbSubsystem.rawFront > 100) {
            ClimbSubsystem.frontWinchPercentOutput = -1.0
        } else {
            ClimbSubsystem.frontWinchPercentOutput = -0.5
        }
    }
}