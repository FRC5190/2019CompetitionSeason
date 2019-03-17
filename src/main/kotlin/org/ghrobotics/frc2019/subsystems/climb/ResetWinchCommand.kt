package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.Source

class ResetWinchCommand(private val resetFront: Boolean) : FalconCommand(ClimbSubsystem) {

    private val selectedSensorPosition: Source<Int>

    init {
        finishCondition += if (resetFront) {
            selectedSensorPosition = ClimbSubsystem::rawFrontWinchPosition
            {
                ClimbSubsystem.isFrontReverseLimitSwitchClosed && ClimbSubsystem.rawFrontWinchPosition < 2000
            }
        } else {
            selectedSensorPosition = ClimbSubsystem::rawBackWinchPosition
            {
                ClimbSubsystem.isBackReverseLimitSwitchClosed && ClimbSubsystem.rawBackWinchPosition < 2000
            }
        }
    }

    override suspend fun initialize() {
        val wantedState = ClimbSubsystem.ClimbLegState.OpenLoop {
            if (selectedSensorPosition() > 100) -1.0 else -0.5
        }
        if (resetFront) {
            ClimbSubsystem.wantedFrontWinchState = wantedState
        } else {
            ClimbSubsystem.wantedBackWinchState = wantedState
        }
    }

    override suspend fun dispose() {
        if (resetFront) {
            ClimbSubsystem.wantedFrontWinchState = ClimbSubsystem.ClimbLegState.Nothing
        } else {
            ClimbSubsystem.wantedBackWinchState = ClimbSubsystem.ClimbLegState.Nothing
        }
    }
}