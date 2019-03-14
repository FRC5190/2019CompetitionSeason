package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source
import kotlin.math.withSign

class OpenLoopElevatorCommand(
    private val percentOutput: DoubleSource
) : FalconCommand(ElevatorSubsystem) {
    constructor(
        percentOutput: Double
    ) : this(Source(percentOutput))

    override suspend fun execute() {
        var output = percentOutput()
        if (!(output epsilonEquals 0.0)) {
            output += (if (ElevatorSubsystem._position < Constants.kElevatorSwitchHeight) {
                Constants.kElevatorBelowSwitchKs
            } else {
                Constants.kElevatorAfterSwitchKs
            }).withSign(output)
        }
        ElevatorSubsystem.setPercentOutput(output)
    }
}