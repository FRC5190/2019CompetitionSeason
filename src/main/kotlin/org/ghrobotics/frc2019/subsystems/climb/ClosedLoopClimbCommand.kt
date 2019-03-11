package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.absoluteValue

class ClosedLoopClimbCommand(
    private val frontTarget: Double,
    private val backTarget: Double
) : FalconCommand(ClimbSubsystem) {

    init {
        finishCondition += {
            (ClimbSubsystem.rawFront - frontTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance &&
                (ClimbSubsystem.rawBack - backTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance
        }
    }

    override suspend fun initialize() {
        ClimbSubsystem.climbToHeight(frontTarget, backTarget)
    }

    override suspend fun dispose() {
//        ClimbSubsystem.zeroOutputs()
    }
}
