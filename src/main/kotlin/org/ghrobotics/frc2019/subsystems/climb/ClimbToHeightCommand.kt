package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length

class ClimbToHeightCommand(private val targetHeight: Length) :
    FalconCommand(ClimbSubsystem) {

    init {
        finishCondition += {
            ClimbSubsystem.frontWinchVelocity < Constants.kClimbWinchClosedLoopVelocityTolerance &&

                ((ClimbSubsystem.backWinchPosition + ClimbSubsystem.frontWinchPosition) / 2.0 - targetHeight).absoluteValue <
                Constants.kClimbWinchClosedLoopTolerance &&

                ClimbSubsystem.backWinchVelocity < Constants.kClimbWinchClosedLoopVelocityTolerance
        }
    }

    override suspend fun initialize() {
        ClimbSubsystem.climbToHeight(targetHeight)
    }
}
