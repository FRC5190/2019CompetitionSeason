package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source

class OpenLoopClimbCommand(private val frontOutput: DoubleSource, private val backOutput: DoubleSource) :
    FalconCommand(ClimbSubsystem) {
    constructor(frontOutput: Double, backOutput: Double) : this(Source(frontOutput), Source(backOutput))

    override suspend fun execute() {
        ClimbSubsystem.frontPercentOutput = frontOutput()
        ClimbSubsystem.backPercentOutput = backOutput()
    }
}