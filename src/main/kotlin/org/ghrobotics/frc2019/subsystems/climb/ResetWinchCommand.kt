package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.and

class ResetWinchCommand(private val winch: ClimbSubsystem.Winch) : FalconCommand(ClimbSubsystem) {
    init {
        finishCondition += winch.motor.sensorCollection::isRevLimitSwitchClosed and {
            winch.motor.selectedSensorPosition < 2000
        }
    }

    override suspend fun execute() {
        winch.motor.percentOutput = if (winch.motor.selectedSensorPosition > 100) -1.0 else -0.5
    }

    override suspend fun dispose() {
        winch.motor.neutralOutput()
    }
}