package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length

class ClosedLoopClimbCommand(private val frontTarget: Length, private val backTarget: Length) :
    FalconCommand(ClimbSubsystem) {

    init {
        finishCondition += {
            (ClimbSubsystem.frontPosition - frontTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance &&
                ClimbSubsystem.frontVelocity < Constants.kClimbWinchClosedLoopVelocityTolerance &&
                (ClimbSubsystem.backPosition - backTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance &&
                ClimbSubsystem.backVelocity < Constants.kClimbWinchClosedLoopVelocityTolerance

        }
    }

    override suspend fun initialize() {
        ClimbSubsystem.frontPosition = frontTarget
        ClimbSubsystem.backPosition = backTarget
    }
}