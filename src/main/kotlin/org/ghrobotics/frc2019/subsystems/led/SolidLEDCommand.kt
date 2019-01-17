package org.ghrobotics.frc2019.subsystems.led

import org.ghrobotics.lib.commands.FalconCommand
import java.awt.Color

class SolidLEDCommand(private val color: Color) : FalconCommand(LEDSubsystem) {
    override suspend fun execute() {
        LEDSubsystem.set(color)
    }
}