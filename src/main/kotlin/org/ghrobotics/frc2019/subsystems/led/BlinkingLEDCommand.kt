package org.ghrobotics.frc2019.subsystems.led

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Time
import java.awt.Color

class BlinkingLEDCommand(
    private val color: Color,
    blinkInterval: Time
) : FalconCommand(LEDSubsystem) {
    private val blinkIntervalMs = blinkInterval.millisecond.toLong()
    private var startTime = 0L

    override suspend fun initialize() {
        startTime = System.currentTimeMillis()
    }

    override suspend fun execute() {
        if ((System.currentTimeMillis() - startTime) % blinkIntervalMs > blinkIntervalMs / 2) {
            LEDSubsystem.set(Color.BLACK)
        } else {
            LEDSubsystem.set(color)
        }
    }
}