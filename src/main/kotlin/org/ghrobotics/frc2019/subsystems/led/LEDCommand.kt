package org.ghrobotics.frc2019.subsystems.led

import org.ghrobotics.lib.commands.FalconCommand

class LEDCommand(private val mode: LEDSubsystem.Mode) : FalconCommand(LEDSubsystem) {

    init {
        this.wrappedValue.setRunWhenDisabled(true)
    }

    override suspend fun initialize() {
        LEDSubsystem.set(mode)
    }

    override suspend fun dispose() {
        LEDSubsystem.set(LEDSubsystem.Mode.DISABLED)
    }
}