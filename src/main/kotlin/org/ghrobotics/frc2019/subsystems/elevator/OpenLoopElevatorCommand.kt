package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source

class OpenLoopElevatorCommand(
    private val percentOutput: DoubleSource
) : FalconCommand(ElevatorSubsystem) {
    constructor(
        percentOutput: Double
    ) : this(Source(percentOutput))

    override suspend fun execute() {
        ElevatorSubsystem.setPercentOutput(percentOutput())
    }
}