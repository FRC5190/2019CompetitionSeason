package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source

class OpenLoopArmCommand(private val percentOutput: DoubleSource) : FalconCommand(ArmSubsystem) {
    constructor(percentOutput: Double) : this(Source(percentOutput))

    override suspend fun initialize() {
        ArmSubsystem.wantedState = ArmSubsystem.ArmState.OpenLoop(percentOutput, true)
    }
}