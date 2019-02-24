package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.absoluteValue

class ClimbToLevel3Command :
    FalconCommand(ClimbSubsystem) {

    init {
        finishCondition += {
            (ClimbSubsystem.rawFront - Constants.kFrontEncoderPositionL3).absoluteValue < 2000 &&
                (ClimbSubsystem.rawBack - Constants.kBackEncoderPositionL3).absoluteValue < 2000
        }
    }

    override suspend fun initialize() {
        ClimbSubsystem.climbToLevel3()
    }

    override suspend fun dispose() {
        ClimbSubsystem.zeroOutputs()
    }

}
