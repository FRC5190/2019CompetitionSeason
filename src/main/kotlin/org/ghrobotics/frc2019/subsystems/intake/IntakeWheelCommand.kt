/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import org.ghrobotics.frc2019.sensors.BallSensors
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import kotlin.math.withSign

class IntakeWheelCommand(
    private val direction: IntakeSubsystem.Direction,
    speed: DoubleSource = Source(1.0)
) : FalconCommand(IntakeSubsystem) {

    private val speed = speed.map {
        it.withSign(if (direction == IntakeSubsystem.Direction.IN) -1 else 1)
    }

    constructor(direction: IntakeSubsystem.Direction, speed: Double) : this(direction, Source(speed))

    init {
        if (direction == IntakeSubsystem.Direction.IN) finishCondition += BallSensors.ballIn
}

    override suspend fun execute() {
        if (!BallSensors.ballIn()) {
            IntakeSubsystem.set(ControlMode.PercentOutput, speed())
        } else {
            IntakeSubsystem.zeroOutputs()
        }
    }

    override suspend fun dispose() {
        IntakeSubsystem.zeroOutputs()
    }
}