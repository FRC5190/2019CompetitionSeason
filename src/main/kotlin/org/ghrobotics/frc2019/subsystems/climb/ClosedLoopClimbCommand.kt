package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length

class ClosedLoopClimbCommand(private val frontTarget: Length, private val backTarget: Length) :
    FalconCommand(ClimbSubsystem) {

    init {
        finishCondition += {
            (ClimbSubsystem.frontWinchPosition - frontTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance &&
                ClimbSubsystem.frontWinchVelocity < Constants.kClimbWinchClosedLoopVelocityTolerance &&
                (ClimbSubsystem.backWinchPosition - backTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance &&
                ClimbSubsystem.backWinchVelocity < Constants.kClimbWinchClosedLoopVelocityTolerance

        }
    }

    override suspend fun initialize() {
        ClimbSubsystem.frontWinchPosition = frontTarget
        ClimbSubsystem.backWinchPosition = backTarget
    }
}
