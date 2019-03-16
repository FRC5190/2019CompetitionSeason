package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source
import kotlin.math.withSign

class OpenLoopElevatorCommand(
    private val useFeedForward: Boolean,
    private val percentOutput: DoubleSource
) : FalconCommand(ElevatorSubsystem) {
    constructor(
        percentOutput: DoubleSource
    ) : this(true, percentOutput)

    constructor(
        percentOutput: Double
    ) : this(Source(percentOutput))

    override suspend fun initialize() {
        ElevatorSubsystem.wantedState = ElevatorSubsystem.ElevatorState.OpenLoop({
            var output = percentOutput()
            if (useFeedForward && !(output epsilonEquals 0.0)) {
                output += (if (ElevatorSubsystem.position < Constants.kElevatorSwitchHeight.value) {
                    Constants.kElevatorBelowSwitchKs
                } else {
                    Constants.kElevatorAfterSwitchKs
                }).withSign(output)
            }
            output
        }, useFeedForward)
    }
}