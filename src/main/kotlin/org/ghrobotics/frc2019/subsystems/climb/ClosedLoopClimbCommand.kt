package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
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
        ClimbSubsystem.frontWinchPosition = frontTarget.nativeUnits
        ClimbSubsystem.backWinchPosition = backTarget.nativeUnits
    }

    override suspend fun dispose() {
        ClimbSubsystem.zeroOutputs()
    }
}
