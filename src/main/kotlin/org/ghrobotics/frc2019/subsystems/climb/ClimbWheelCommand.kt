package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class ClimbWheelCommand(val percent: DoubleSource) : FalconCommand() {

    override suspend fun execute() {
        ClimbSubsystem.wantedWheelPercentOutput = percent()
    }

    override suspend fun dispose() {
        ClimbSubsystem.wantedWheelPercentOutput = 0.0
    }
}