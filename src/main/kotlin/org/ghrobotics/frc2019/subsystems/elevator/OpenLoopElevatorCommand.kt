package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source

class OpenLoopElevatorCommand(
    private val percentOutput: DoubleSource,
    private val useFeedForward: Boolean = true
) : FalconCommand(ElevatorSubsystem) {
    constructor(
        percentOutput: Double,
        useFeedForward: Boolean = true
    ) : this(Source(percentOutput), useFeedForward)

    override suspend fun execute() {
        ElevatorSubsystem.percentOutput = percentOutput() + if(useFeedForward) Constants.kElevatorAfterSwitchKg else 0.0
    }
}