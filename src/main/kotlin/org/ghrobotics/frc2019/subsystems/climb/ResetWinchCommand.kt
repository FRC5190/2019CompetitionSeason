package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand

class ResetWinchCommand(val winch: ClimbSubsystem.Winch) : FalconCommand(ClimbSubsystem) {
    init {
        finishCondition += winch.motor.sensorCollection::isRevLimitSwitchClosed
    }

    override suspend fun execute() {
        winch.motor.percentOutput = if (winch.motor.selectedSensorPosition > 100) -1.0 else -0.5
    }

    override suspend fun dispose() {
        winch.motor.neutralOutput()
    }
}