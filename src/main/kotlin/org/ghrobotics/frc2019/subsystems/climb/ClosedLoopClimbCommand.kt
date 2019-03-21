package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length
import kotlin.math.absoluteValue

class ClosedLoopClimbCommand(
    frontTarget: Length,
    backTarget: Length
) : FalconCommand(ClimbSubsystem) {

    private val frontTarget = frontTarget.value
    private val backTarget = backTarget.value

    init {
        finishCondition += {
            (ClimbSubsystem.frontWinchPosition - this.frontTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance.value &&
                (ClimbSubsystem.backWinchPosition - this.backTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance.value
        }
    }

    override suspend fun initialize() {
        ClimbSubsystem.wantedFrontWinchState = ClimbSubsystem.ClimbLegState.Climb(frontTarget)
        ClimbSubsystem.wantedBackWinchState = ClimbSubsystem.ClimbLegState.Climb(backTarget)
    }
}
